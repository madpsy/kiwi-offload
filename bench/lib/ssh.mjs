// One multiplexed ssh connection to the Kiwi, opened once and reused for every command (a new ssh session costs the
// Beagle's single core noticeably). The bench uses it only between runs: to write the configuration and restart
// services. Nothing is measured over ssh.
//
// Authentication: the user's ssh keys if they work, else the password from the credentials file (through
// SSH_ASKPASS, so no extra tool is needed). sudo reads the same password on stdin.
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';

export class Ssh {
    constructor({ host, user, pass, port }) {
        this.host = host; this.user = user; this.pass = pass || ''; this.port = port || 22;
        this.dir = fs.mkdtempSync(path.join(os.tmpdir(), 'kbench-'));
        fs.chmodSync(this.dir, 0o700);
        this.ctl = path.join(this.dir, 'ctl');
        this.askpass = path.join(this.dir, 'askpass.sh');
        fs.writeFileSync(this.askpass, '#!/bin/sh\nprintf \'%s\\n\' "$KBENCH_SSH_PASS"\n', { mode: 0o700 });
    }
    base() {
        return ['-x', '-p', String(this.port), '-o', 'ControlMaster=auto', '-o', `ControlPath=${this.ctl}`, '-o', 'ControlPersist=900',
                '-o', 'StrictHostKeyChecking=accept-new', '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=30',
                '-o', 'LogLevel=ERROR', `${this.user}@${this.host}`];
    }
    env() {
        const e = { ...process.env, KBENCH_SSH_PASS: this.pass };
        if (this.pass) { e.SSH_ASKPASS = this.askpass; e.SSH_ASKPASS_REQUIRE = 'force'; e.DISPLAY = e.DISPLAY || ':0'; }
        return e;
    }
    // run a command; resolves { code, out, err }
    exec(cmd, { input = null, timeout = 120000 } = {}) {
        return new Promise(resolve => {
            const p = spawn('ssh', [...this.base(), cmd], { env: this.env(), stdio: ['pipe', 'pipe', 'pipe'] });
            let out = '', err = '';
            const to = setTimeout(() => { p.kill('SIGKILL'); err += '\n(timeout)'; }, timeout);
            p.stdout.on('data', d => out += d); p.stderr.on('data', d => err += d);
            p.on('close', code => { clearTimeout(to); resolve({ code, out, err: err.replace(/^.*(Debian|BeagleBoard|bbb\.io|default username).*\n?/gm, '') }); });
            if (input != null) p.stdin.end(input); else p.stdin.end();
        });
    }
    async check(cmd, opts) {
        const r = await this.exec(cmd, opts);
        if (r.code !== 0) throw new Error(`ssh: '${cmd.slice(0, 80)}' failed (${r.code}): ${r.err.trim().slice(0, 300)}`);
        return r.out;
    }
    // a command as root: sudo with the password on stdin (or passwordless sudo)
    sudo(cmd, opts = {}) {
        const c = this.pass ? `sudo -S -p '' sh -c ${shq(cmd)}` : `sudo -n sh -c ${shq(cmd)}`;
        return this.check(c, { ...opts, input: this.pass ? this.pass + '\n' + (opts.input || '') : opts.input });
    }
    async open() { await this.check('true'); }
    close() {
        return new Promise(resolve => {
            const p = spawn('ssh', ['-o', `ControlPath=${this.ctl}`, '-O', 'exit', `${this.user}@${this.host}`], { stdio: 'ignore' });
            p.on('close', () => { try { fs.rmSync(this.dir, { recursive: true, force: true }); } catch {} resolve(); });
        });
    }
}
export const shq = s => `'${String(s).replace(/'/g, `'\\''`)}'`;
