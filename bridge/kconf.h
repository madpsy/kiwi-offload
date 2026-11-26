// kconf.h: the ONE configuration file of kiwi-offload (e.g. /home/debian/ko/kiwi-offload.conf), read by both kbridge
// (C) and kweb (C++) with this same parser, so each knows the other's settings (kweb honours [pool], kbridge can check
// [kweb]); see config/kiwi-offload.conf.example for every key.
//
// Format: INI. [section] lines, "key = value" lines, '#' or ';' starts a comment, blank lines ignored. Sections:
//   [kbridge]      streaming settings (each also settable by its KB_* environment variable, which wins)
//   [host]         the host's slices (slices = <kHz>:<R>[:<gain>][:L][:b<bits>] ...)
//   [pool]         how the 13 DDCs are shared between the host and the web users
//   [kweb]         the web SDR (command-line --options win)
//   [window NAME]  a fixed shared window offered to web users (e.g. a ham band)
#ifndef KCONF_H
#define KCONF_H
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef KCONF_ERR                           // where syntax errors go (kweb also shows them on its admin page)
#define KCONF_ERR(...) fprintf(stderr, __VA_ARGS__)
#endif
#define KCONF_MAX 512
typedef struct { char sec[64], key[48], val[256]; int line; } kconf_ent_t;
typedef struct { int n, errors; kconf_ent_t e[KCONF_MAX]; char path[256]; } kconf_t;

static inline char *kconf_trim(char *s)
{
    while (*s && isspace((unsigned char) *s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char) e[-1])) *--e = 0;
    return s;
}
// returns 1 if the file was read (syntax errors are printed and counted in c->errors)
static inline int kconf_load(kconf_t *c, const char *path)
{
    memset(c, 0, sizeof *c);
    snprintf(c->path, sizeof c->path, "%s", path);
    FILE *f = fopen(path, "r");
    if (!f) { KCONF_ERR("config: cannot read %s\n", path); return 0; }
    char line[512], sec[64] = ""; int ln = 0;
    while (fgets(line, sizeof line, f)) {
        ln++;
        char *h = strpbrk(line, "#;"); if (h) *h = 0;
        char *l = kconf_trim(line);
        if (!*l) continue;
        if (*l == '[') {
            char *r = strchr(l, ']');
            if (!r) { KCONF_ERR("%s:%d: missing ]\n", path, ln); c->errors++; continue; }
            *r = 0; snprintf(sec, sizeof sec, "%s", kconf_trim(l + 1));
            if (c->n < KCONF_MAX) { kconf_ent_t *e = &c->e[c->n++]; snprintf(e->sec, sizeof e->sec, "%s", sec); e->key[0] = 0; e->line = ln; }
            continue;                                   // (an entry with no key marks the section, e.g. an empty window)
        }
        char *eq = strchr(l, '=');
        if (!eq) { KCONF_ERR("%s:%d: expected key = value\n", path, ln); c->errors++; continue; }
        *eq = 0;
        if (c->n >= KCONF_MAX) { KCONF_ERR("%s: too many entries\n", path); c->errors++; break; }
        kconf_ent_t *e = &c->e[c->n++];
        snprintf(e->sec, sizeof e->sec, "%s", sec);
        snprintf(e->key, sizeof e->key, "%s", kconf_trim(l));
        snprintf(e->val, sizeof e->val, "%s", kconf_trim(eq + 1));
        e->line = ln;
    }
    fclose(f);
    return 1;
}
static inline const char *kconf_get(const kconf_t *c, const char *sec, const char *key)   // the last one wins
{
    for (int i = c->n - 1; i >= 0; i--) if (!strcmp(c->e[i].sec, sec) && !strcmp(c->e[i].key, key)) return c->e[i].val;
    return NULL;
}
static inline int kconf_int(const kconf_t *c, const char *sec, const char *key, int dflt)
{
    const char *v = kconf_get(c, sec, key); return v ? atoi(v) : dflt;
}
static inline int kconf_on(const char *v)          // on/yes/true/1
{
    return v && (!strcmp(v, "1") || !strcmp(v, "on") || !strcmp(v, "yes") || !strcmp(v, "true"));
}
#endif
