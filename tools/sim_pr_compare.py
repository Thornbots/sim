#!/usr/bin/env python3
# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Run the sim suites on a PR's base and head and diff the results.

Host-side: `gh`/`git` here, builds and runs in the dev container via dexec.sh.
`sim_pr_compare.py Thornbots/sim#2 [--suite drift,shot_hit] [--repeat 3]`.
Each side is a colcon workspace of package worktrees under
<ws>/worktrees/sim-pr/<id>/; results and report.md land in its results/.
Base is the PR's merge-base, so only the PR's own changes differ.
See README.md's "Comparing a PR" for the design.
"""
import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ORG = 'Thornbots'
SUPER_REPO = 'thornbots_workspace'
# Submodule paths the suites load code from; every one is built per side so
# nothing falls through to the user's own install/.
PACKAGES = ('sim', 'thornbots_pkg', 'sentry_localization', 'rf2o_laser_odometry',
            'ros2_dji_serial_bridge', 'sllidar_ros2')
BUILD_TARGETS = ('sim', 'thornbots_pkg')
OVERRIDES = ('sim', 'thornbots_pkg', 'sentry_localization', 'rf2o_laser_odometry',
             'dji_serial_bridge', 'sllidar_ros2')
CONTAINER_WS = '/workspaces/isaac_ros-dev'
# launch file and extra launch args per suite.
SUITES = {
    'drift': ('localization_tests.launch.py', {'suite': 'drift'}),
    'ekf': ('localization_tests.launch.py', {'suite': 'ekf'}),
    'shot_hit': ('shot_hit.launch.py', {}),
}
RUN_TIMEOUT_S = 45 * 60
LIVE_SESSION = r'ign gazebo|gz sim|slam_toolbox|amcl|ekf_filter_node|pose_emulator|ros2 launch'


def sh(cmd, **kw):
    kw.setdefault('check', True)
    kw.setdefault('text', True)
    kw.setdefault('capture_output', True)
    return subprocess.run(cmd, **kw)


def git(repo, *args):
    return sh(['git', '-C', repo, *args]).stdout.strip()


def parse_pr(spec):
    m = (re.match(r'https://github\.com/([^/]+)/([^/]+)/pull/(\d+)', spec)
         or re.match(r'(?:([^/#]+)/)?([^/#]+)#(\d+)$', spec))
    if not m:
        sys.exit(f'cannot parse PR {spec!r}; use a URL or [owner/]repo#N')
    return (m.group(1) or ORG), m.group(2), int(m.group(3))


def submodule_paths(super_repo):
    """Map repo name -> submodule path, from .gitmodules."""
    out = git(super_repo, 'config', '-f', '.gitmodules', '--get-regexp', r'\.url$')
    paths = {}
    for line in out.splitlines():
        key, url = line.split()
        name = key[len('submodule.'):-len('.url')]
        path = git(super_repo, 'config', '-f', '.gitmodules', f'submodule.{name}.path')
        paths[url.rstrip('/').removesuffix('.git').rsplit('/', 1)[-1]] = path
    return paths


def fetch_pr(repo_dir, number, base_oid, head_oid):
    git(repo_dir, 'fetch', '-q', 'origin', f'pull/{number}/head', base_oid)
    return git(repo_dir, 'merge-base', base_oid, head_oid)


def gitlinks(super_repo, rev):
    return {p: git(super_repo, 'rev-parse', f'{rev}:{p}') for p in PACKAGES}


def resolve(args):
    """Return (pr dict, {'base': {path: sha}, 'head': {path: sha}})."""
    owner, repo, number = parse_pr(args.pr)
    pr = json.loads(sh(['gh', 'pr', 'view', str(number), '-R', f'{owner}/{repo}', '--json',
                        'number,title,url,baseRefOid,headRefOid,baseRefName']).stdout)
    pr['repo'] = f'{owner}/{repo}'
    super_repo = args.super_repo
    if repo == SUPER_REPO:
        base_super = fetch_pr(super_repo, number, pr['baseRefOid'], pr['headRefOid'])
        sides = {'base': gitlinks(super_repo, base_super),
                 'head': gitlinks(super_repo, pr['headRefOid'])}
    else:
        path = submodule_paths(super_repo).get(repo)
        if path not in PACKAGES:
            sys.exit(f'{repo} is not one of the packages the sim suites run: {PACKAGES}')
        git(super_repo, 'fetch', '-q', 'origin', 'main')
        links = gitlinks(super_repo, 'origin/main')
        base_pkg = fetch_pr(os.path.join(super_repo, path), number,
                            pr['baseRefOid'], pr['headRefOid'])
        sides = {'base': {**links, path: base_pkg},
                 'head': {**links, path: pr['headRefOid']}}
    return pr, sides


def checkout(super_repo, side_dir, links):
    """Put each package at its sha in side_dir/src as a worktree of the main checkout's repo."""
    for path, sha in links.items():
        repo = os.path.join(super_repo, path)
        dest = os.path.join(side_dir, 'src', path)
        if os.path.exists(dest):
            if git(dest, 'rev-parse', 'HEAD') == sha:
                continue
            git(repo, 'worktree', 'remove', '--force', dest)
        if sh(['git', '-C', repo, 'cat-file', '-e', f'{sha}^{{commit}}'], check=False).returncode:
            git(repo, 'fetch', '-q', 'origin', sha)
        git(repo, 'worktree', 'add', '-q', '--detach', dest, sha)


def remove_worktrees(super_repo, side_dir):
    for path in PACKAGES:
        dest = os.path.join(side_dir, 'src', path)
        if os.path.exists(dest):
            git(os.path.join(super_repo, path), 'worktree', 'remove', '--force', dest)


class Container:
    """dexec.sh against the user's running container; never starts one."""

    def __init__(self, ws):
        self.ws = ws
        self.dexec = os.path.join(ws, 'src', 'isaac_ros_common', 'scripts', 'dexec.sh')

    def path(self, host_path):
        return CONTAINER_WS + os.path.abspath(host_path)[len(self.ws):]

    def run(self, script, workdir, log=None, check=True):
        cmd = [self.dexec, '-w', self.path(workdir), '--', 'bash', '-c', script]
        if log is None:
            return sh(cmd, check=check)
        with open(log, 'w') as f:
            return subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT,
                                  stdin=subprocess.DEVNULL, check=check)

    def live_session(self):
        out = self.run('ps -eo pid,cmd', self.ws).stdout
        return [line for line in out.splitlines()
                if re.search(LIVE_SESSION, line) and 'dexec' not in line]


def build(box, side_dir, log):
    script = (f'colcon build --symlink-install --packages-up-to {" ".join(BUILD_TARGETS)} '
              f'--allow-overriding {" ".join(OVERRIDES)}')
    if box.run(script, side_dir, log, check=False).returncode:
        sys.exit(f'build failed in {side_dir}; see {log}')


def run_suite(box, side_dir, suite, out_stem, args):
    """Run one suite on one side; the junit XML is the result, the log is for reading."""
    launch, extra = SUITES[suite]
    junit, log = out_stem + '.xml', out_stem + '.log'
    launch_args = {**extra, 'headless': str(not args.gui).lower(),
                   'real_time_factor': args.real_time_factor,
                   'pytest_args': f'-o junit_family=xunit1 --junitxml={box.path(junit)}'}
    logs = out_stem + '_logs'
    if suite == 'shot_hit':
        launch_args['log_dir'] = box.path(logs)
    pidfile = box.path(out_stem + '.pid')
    script = ('source install/setup.bash && echo $$ > ' + shlex.quote(pidfile) + ' && '
              f'exec timeout -s INT -k 60 {RUN_TIMEOUT_S} ros2 launch sim {launch} '
              + ' '.join(shlex.quote(f'{k}:={v}') for k, v in launch_args.items()))
    started = time.monotonic()
    try:
        box.run(script, side_dir, log, check=False)
    except KeyboardInterrupt:
        # docker exec doesn't forward the signal; timeout passes SIGINT on to the launch.
        box.run(f'kill -INT $(cat {shlex.quote(pidfile)})', side_dir, check=False)
        raise
    if suite != 'shot_hit':
        box.run(f'cp -r /tmp/localization_drift_tests {shlex.quote(box.path(logs))}',
                side_dir, check=False)
    print(f'  {os.path.basename(out_stem)}: {time.monotonic() - started:.0f}s'
          + ('' if os.path.exists(junit) else ' -- NO RESULT, see ' + log))


def read_junit(path):
    """Return {test id: (outcome, {metric: float})}; {} if the run left no XML."""
    if not os.path.exists(path):
        return {}
    cases = {}
    for tc in ET.parse(path).iter('testcase'):
        outcome = 'pass'
        for tag, name in (('failure', 'FAIL'), ('error', 'ERROR'), ('skipped', 'skip')):
            if tc.find(tag) is not None:
                outcome = name
        metrics = {}
        for prop in tc.iter('property'):
            try:
                metrics[prop.get('name')] = float(prop.get('value'))
            except (TypeError, ValueError):
                pass
        cases[tc.get('name')] = (outcome, metrics)
    return cases


def fmt(values):
    values = [v for v in values if v == v]  # drop NaN
    if not values:
        return '-'
    mean = sum(values) / len(values)
    spread = f' ({min(values):.3g}..{max(values):.3g})' if min(values) != max(values) else ''
    return f'{mean:.3g}{spread}'


def report(pr, sides, run_dir, suites, repeat):
    results = os.path.join(run_dir, 'results')
    changed = [p for p in PACKAGES if sides['base'][p] != sides['head'][p]]
    lines = [f'### Sim results: {pr["repo"]}#{pr["number"]}', '',
             f'{pr["title"]}. {repeat} run(s) per side; base is the merge-base.', '']
    lines += [f'- `{p}`: base `{sides["base"][p][:8]}`, head `{sides["head"][p][:8]}`'
              for p in changed]
    for suite in suites:
        per_side = {side: [read_junit(os.path.join(results, f'{side}-{suite}-{i}.xml'))
                           for i in range(repeat)] for side in ('base', 'head')}
        tests = sorted({t for runs in per_side.values() for r in runs for t in r})
        lines += ['', f'#### {suite}', '',
                  '| test | base | head | metric | base | head | change |',
                  '|---|---|---|---|---|---|---|']
        if not tests:
            lines.append('| (no results on either side; see the logs) | | | | | | |')
        for t in tests:
            verdict = {s: ' '.join(r[t][0] if t in r else 'none' for r in per_side[s])
                       for s in per_side}
            names = sorted({m for runs in per_side.values() for r in runs if t in r
                            for m in r[t][1]}, key=lambda m: (m == 'log_errors', m))
            rows = []
            for m in names:
                vals = {s: [r[t][1][m] for r in per_side[s] if t in r and m in r[t][1]]
                        for s in per_side}
                b, h = (sum(v) / len(v) if v else None for v in vals.values())
                delta = f'{h - b:+.3g}' if b is not None and h is not None else '-'
                rows.append((m, fmt(vals['base']), fmt(vals['head']), delta))
            first = rows.pop(0) if rows else ('', '', '', '')
            lines.append(f'| {t} | {verdict["base"]} | {verdict["head"]} | ' + ' | '.join(first)
                         + ' |')
            lines += [f'| | | | {" | ".join(r)} |' for r in rows]
    lines += ['', 'Verdicts are per run, in run order; `none` means that run left no result '
              '(crashed or timed out). Metrics are the mean over runs, range in brackets. '
              'Single runs are noisy, shot-hit especially. Made by '
              '`sim/tools/sim_pr_compare.py`.']
    text = '\n'.join(lines) + '\n'
    with open(os.path.join(run_dir, 'report.md'), 'w') as f:
        f.write(text)
    return text


def main():
    ws_default = os.path.expanduser('~/workspaces/isaac_ros-dev')
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('pr', help='PR URL or [owner/]repo#N, on thornbots_workspace or a package')
    p.add_argument('--suite', default='drift,shot_hit',
                   help=f'comma-separated, from {",".join(SUITES)} (default drift,shot_hit)')
    p.add_argument('--repeat', type=int, default=1,
                   help='runs per side, alternating which side goes first')
    p.add_argument('--gui', action='store_true', help='show gz and rviz (default headless)')
    p.add_argument('--real-time-factor', default='0',
                   help='sim speed for every suite; 0 (default) is unthrottled, 1 real time')
    p.add_argument('--ws', default=ws_default, help='host workspace root, mounted at '
                   f'{CONTAINER_WS} in the container (default {ws_default})')
    p.add_argument('--no-build', action='store_true', help='reuse each side\'s install/')
    p.add_argument('--report-only', action='store_true', help='re-render report.md, run nothing')
    p.add_argument('--comment', action='store_true', help='post report.md on the PR')
    p.add_argument('--clean', action='store_true',
                   help='remove the package worktrees and builds afterwards, keep results/')
    args = p.parse_args()
    args.super_repo = os.path.join(args.ws, 'src')
    suites = args.suite.split(',')
    for s in suites:
        if s not in SUITES:
            sys.exit(f'unknown suite {s!r}; one of {list(SUITES)}')

    pr, sides = resolve(args)
    run_id = f'{pr["repo"].split("/")[1]}-{pr["number"]}'
    run_dir = os.path.join(args.ws, 'worktrees', 'sim-pr', run_id)
    results = os.path.join(run_dir, 'results')
    os.makedirs(results, exist_ok=True)
    with open(os.path.join(results, 'sides.json'), 'w') as f:
        json.dump(sides, f, indent=2)
    print(f'{pr["repo"]}#{pr["number"]}: {pr["title"]}\n  run dir {run_dir}')

    if not args.report_only:
        box = Container(args.ws)
        live = box.live_session()
        if live:
            sys.exit('a sim or robot stack is already running in the container; it would '
                     'corrupt both runs. Stop it first:\n  ' + '\n  '.join(live))
        for side, links in sides.items():
            side_dir = os.path.join(run_dir, side)
            checkout(args.super_repo, side_dir, links)
            if not args.no_build:
                print(f'building {side} ...')
                build(box, side_dir, os.path.join(results, f'build-{side}.log'))
        for i in range(args.repeat):
            order = ('base', 'head') if i % 2 == 0 else ('head', 'base')
            for suite in suites:
                for side in order:
                    stem = os.path.join(results, f'{side}-{suite}-{i}')
                    shutil.rmtree(stem + '_logs', ignore_errors=True)
                    if os.path.exists(stem + '.xml'):
                        os.remove(stem + '.xml')
                    run_suite(box, os.path.join(run_dir, side), suite, stem, args)
        if args.clean:
            for side in sides:
                remove_worktrees(args.super_repo, os.path.join(run_dir, side))
                for d in ('build', 'install', 'log', 'src'):
                    shutil.rmtree(os.path.join(run_dir, side, d), ignore_errors=True)

    text = report(pr, sides, run_dir, suites, args.repeat)
    print('\n' + text)
    if args.comment:
        sh(['gh', 'pr', 'comment', str(pr['number']), '-R', pr['repo'],
            '--body-file', os.path.join(run_dir, 'report.md')], capture_output=False)


if __name__ == '__main__':
    main()
