// kodma: two 4 MB DMA rings for the PRU -> DDR streams, mmap-able from user space.
//
// Default (cached=1): each ring is ordinary cacheable RAM (alloc_pages, physically contiguous), DMA-mapped once. User
// space maps both rings CACHEABLE at offsets 0 and 4 MB and, before reading a range the PRU has written, invalidates it
// with ioctl(KODMA_INVAL) (dma_sync_single_for_cpu: by-line cache invalidate). Cached loads are fast and NEON is safe.
// The ARM never writes the rings, so no line is ever dirty.
//
// Why not the old uncached (dma_alloc_coherent) mapping: NEON loads from Normal non-cacheable memory HANG the
// Cortex-A8 (the ARM stops dead, no oops, the PRUs keep running); plain 32-bit uncached loads are safe but cost ~half
// the CPU at 77 Mbit/s. cached=0 still gives that buffer (one 8 MB block, rings at +0/+4 MB); never use NEON on it.
//
// Bus/physical addresses for the PRUs: /sys/module/kodma/parameters/phys (ring 0) and phys1 (ring 1); "cached" says
// which mode is active (if the 4 MB allocations fail at load, the module falls back to cached=0).
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#define RING_BYTES  (4ul << 20)
#define RING_ORDER  10                      // 2^10 pages = 4 MB
struct kodma_range { __u32 off, len; };     // byte range within the 8 MB mapping (ring = off / 4 MB; may not cross)
#define KODMA_INVAL _IOW('k', 1, struct kodma_range)

static int cached = 1;
module_param(cached, int, 0444);
MODULE_PARM_DESC(cached, "1 = cacheable rings + KODMA_INVAL (default); 0 = one uncached coherent buffer (no NEON!)");
static unsigned long phys, phys1;
module_param(phys, ulong, 0444);
MODULE_PARM_DESC(phys, "bus address of ring 0 (read-only)");
module_param(phys1, ulong, 0444);
MODULE_PARM_DESC(phys1, "bus address of ring 1 (read-only)");

static struct platform_device *pdev;
static struct page *pg[2];
static dma_addr_t bus[2];
static void *coh;                           // cached=0 buffer (8 MB)
static dma_addr_t coh_bus;

static int kodma_mmap(struct file *f, struct vm_area_struct *vma)
{
    unsigned long len = vma->vm_end - vma->vm_start;
    int i, r;
    if (vma->vm_pgoff || len > 2 * RING_BYTES) return -EINVAL;
    if (!cached) return dma_mmap_coherent(&pdev->dev, vma, coh, coh_bus, len);
    for (i = 0; i < 2 && len > i * RING_BYTES; i++) {
        unsigned long n = min(len - i * RING_BYTES, RING_BYTES);
        r = remap_pfn_range(vma, vma->vm_start + i * RING_BYTES, page_to_pfn(pg[i]), n, vma->vm_page_prot);
        if (r) return r;
    }
    return 0;
}

static long kodma_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    struct kodma_range rg;
    unsigned int i;
    if (cmd != KODMA_INVAL) return -ENOTTY;
    if (!cached) return 0;
    if (copy_from_user(&rg, (void __user *) arg, sizeof rg)) return -EFAULT;
    i = rg.off / RING_BYTES;
    if (i > 1 || !rg.len || (rg.off % RING_BYTES) + rg.len > RING_BYTES) return -EINVAL;
    dma_sync_single_for_cpu(&pdev->dev, bus[i] + rg.off % RING_BYTES, rg.len, DMA_BIDIRECTIONAL);
    return 0;
}

static const struct file_operations kodma_fops = {
    .owner = THIS_MODULE, .mmap = kodma_mmap, .unlocked_ioctl = kodma_ioctl,
};
static struct miscdevice kodma_misc = { .minor = MISC_DYNAMIC_MINOR, .name = "kodma", .fops = &kodma_fops, .mode = 0600 };

static void free_rings(void)
{
    int i;
    for (i = 0; i < 2; i++) {
        if (!pg[i]) continue;
        if (bus[i]) dma_unmap_page(&pdev->dev, bus[i], RING_BYTES, DMA_BIDIRECTIONAL);
        __free_pages(pg[i], RING_ORDER);
        pg[i] = NULL; bus[i] = 0;
    }
}

static int alloc_rings(void)
{
    int i;
    for (i = 0; i < 2; i++) {
        pg[i] = alloc_pages(GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN, RING_ORDER);
        if (!pg[i]) goto fail;
        // map: cleans the zeroes out to DDR and invalidates, so the rings start as zeroes on both sides
        bus[i] = dma_map_page(&pdev->dev, pg[i], 0, RING_BYTES, DMA_BIDIRECTIONAL);
        if (dma_mapping_error(&pdev->dev, bus[i])) { bus[i] = 0; goto fail; }
    }
    phys = bus[0]; phys1 = bus[1];
    return 0;
fail:
    free_rings();
    return -ENOMEM;
}

static int __init kodma_init(void)
{
    int r;
    pdev = platform_device_register_simple("kodma", -1, NULL, 0);
    if (IS_ERR(pdev)) return PTR_ERR(pdev);
    r = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    if (r) goto fail_dev;
    if (cached && alloc_rings()) {
        pr_warn("kodma: 4 MB cacheable rings unavailable, falling back to the uncached buffer\n");
        cached = 0;
    }
    if (!cached) {
        coh = dma_alloc_coherent(&pdev->dev, 2 * RING_BYTES, &coh_bus, GFP_KERNEL);
        if (!coh) { r = -ENOMEM; goto fail_dev; }
        memset(coh, 0, 2 * RING_BYTES);
        phys = coh_bus; phys1 = coh_bus + RING_BYTES;
    }
    r = misc_register(&kodma_misc);
    if (r) goto fail_mem;
    pr_info("kodma: rings at bus 0x%08lx / 0x%08lx (%s)\n", phys, phys1, cached ? "cacheable + KODMA_INVAL" : "uncached");
    return 0;
fail_mem:
    if (coh) dma_free_coherent(&pdev->dev, 2 * RING_BYTES, coh, coh_bus);
    free_rings();
fail_dev:
    platform_device_unregister(pdev);
    return r;
}

static void __exit kodma_exit(void)
{
    misc_deregister(&kodma_misc);
    if (coh) dma_free_coherent(&pdev->dev, 2 * RING_BYTES, coh, coh_bus);
    free_rings();
    platform_device_unregister(pdev);
}

module_init(kodma_init);
module_exit(kodma_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("kiwi-offload: PRU DMA rings (cacheable + invalidate, or uncached)");
