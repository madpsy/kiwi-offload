// The Kiwi's configuration and services, through the one ssh connection. Used only between runs.
//
// Safety: the first thing the bench does is keep the original configuration beside it on the Kiwi
// (<conf>.kbench-orig) and in the results directory. restore() puts it back and restarts what needs it; the bench
// calls it on every exit path. If a previous bench was killed before restoring, the .kbench-orig left behind IS the
// original, and it is used as such.
import { Ssh, shq } from './ssh.mjs';
import { kbridgeSignature } from './conf.mjs';
import { log } from './util.mjs';

export class Kiwi {
    constructor(o) {
        this.o = o;
        this.ssh = new Ssh({ host: o.host, user: o.user, pass: o.pass, port: o.sshPort });
        this.conf = o.conf; this.orig = null; this.current = null;
    }
    async start() {
        await this.ssh.open();
        const c = shq(this.conf), b = shq(this.conf + '.kbench-orig');
        const r = await this.ssh.check(`if [ -f ${b} ]; then echo LEFTOVER; cat ${b}; else cp -p ${c} ${b} && echo FRESH && cat ${c}; fi`);
        const nl = r.indexOf('\n');
        if (r.slice(0, nl) === 'LEFTOVER') log('note: a previous bench did not restore the configuration: its saved original is used');
        this.orig = r.slice(nl + 1);
        this.current = await this.ssh.check(`cat ${c}`);
        this.version = (await this.ssh.exec(`cat ${shq(this.o.dir + '/VERSION')} 2>/dev/null`)).out.trim() || null;
        return this.orig;
    }
    // write a configuration (if different); returns whether kbridge must restart for it
    async write(text) {
        if (text === this.current) return { changed: false, kbridge: false };
        const kb = kbridgeSignature(text) !== kbridgeSignature(this.current);
        const c = shq(this.conf), n = shq(this.conf + '.kbench-new');
        await this.ssh.check(`cat > ${n} && mv ${n} ${c}`, { input: text });
        this.current = text;
        return { changed: true, kbridge: kb };
    }
    async restartKbridge() { log('restarting kbridge (FPGA reload)'); await this.ssh.sudo(`systemctl restart ${this.o.kbridgeService}`, { timeout: 180000 }); }
    async restartKweb() { await this.ssh.sudo(`systemctl restart ${this.o.kwebService}`, { timeout: 60000 }); }
    async restore() {
        if (this.orig == null) return;
        log('restoring the original configuration');
        const w = await this.write(this.orig);
        if (w.kbridge) await this.restartKbridge();
        await this.restartKweb();
        await this.ssh.check(`rm -f ${shq(this.conf + '.kbench-orig')}`);
        this.orig = null;
    }
    close() { return this.ssh.close(); }
}
