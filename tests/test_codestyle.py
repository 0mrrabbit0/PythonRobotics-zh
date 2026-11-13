"""
用 ruff 检查差异代码风格

本代码来自：
https://github.com/scipy/scipy/blob/main/tools/lint.py

Scipy 许可证: https://github.com/scipy/scipy/blob/main/LICENSE.txt
版权所有 (c) 2001-2002 Enthought, Inc. 2003-2022, SciPy Developers.
保留所有权利。

在满足下列条件的前提下，允许以源代码或二进制形式再分发和使用，
无论是否修改：

1. 再分发源代码时必须保留上述版权
   声明、本条件列表以及下列免责声明。

2. 以二进制形式再分发时，必须在文档和/或
   随分发提供的其他材料中复制上述版权声明、
   本条件列表以及下列免责声明。

3. 未经事先书面特别许可，不得使用版权持有人的名称
   或其贡献者的名称来认可或推广由本软件衍生的产品。

本软件由版权持有人与贡献者“按原样”提供，
不提供任何明示或暗示的担保，包括但不限于
对适销性和特定用途适用性的暗示担保。
在任何情况下，版权所有者或贡献者均不对任何直接、间接、
附带、特殊、惩戒性或后果性损害承担责任
（包括但不限于采购替代商品或服务；使用损失、
数据或利润损失；或业务中断），无论其成因为何，
也无论是基于合同、严格责任或侵权
（包括过失或其他）因使用本软件而以任何方式产生的责任，
即使已被告知发生此类损害的可能性。
"""
import conftest
import os
import subprocess


CONFIG = os.path.join(
    os.path.abspath(os.path.dirname(os.path.dirname(__file__))),
    'ruff.toml',
)

ROOT_DIR = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))


def run_ruff(files, fix):
    if not files:
        return 0, ""
    args = ['--fix'] if fix else []
    res = subprocess.run(
        ['ruff', 'check', f'--config={CONFIG}'] + args + files,
        stdout=subprocess.PIPE,
        encoding='utf-8'
    )
    return res.returncode, res.stdout


def rev_list(branch, num_commits):
    """按时间倒序列出提交。
    只显示前 `num_commits` 个。
    """
    res = subprocess.run(
        [
            'git',
            'rev-list',
            '--max-count',
            f'{num_commits}',
            '--first-parent',
            branch
        ],
        stdout=subprocess.PIPE,
        encoding='utf-8',
    )
    res.check_returncode()
    return res.stdout.rstrip('\n').split('\n')


def find_branch_point(branch):
    """查找当前分支从给定分支分出的位置。
    方法基于这篇 Stackoverflow 帖子：
    https://stackoverflow.com/questions/1527234/finding-a-branch-point-with-git#4991675
    """
    branch_commits = rev_list('HEAD', 1000)
    main_commits = set(rev_list(branch, 1000))
    for branch_commit in branch_commits:
        if branch_commit in main_commits:
            return branch_commit

    # 若分支在 1000 次提交之前分出，将无法找到
    # 该祖先。
    raise RuntimeError(
        'Failed to find a common ancestor in the last 1000 commits')


def find_diff(sha):
    """查找自给定 sha 以来的差异。"""
    files = ['*.py']
    res = subprocess.run(
        ['git', 'diff', '--unified=0', sha, '--'] + files,
        stdout=subprocess.PIPE,
        encoding='utf-8'
    )
    res.check_returncode()
    return res.stdout


def diff_files(sha):
    """查找自给定 SHA 以来的差异。"""
    res = subprocess.run(
        ['git', 'diff', '--name-only', '--diff-filter=ACMR', '-z', sha, '--',
         '*.py', '*.pyx', '*.pxd', '*.pxi'],
        stdout=subprocess.PIPE,
        encoding='utf-8'
    )
    res.check_returncode()
    return [os.path.join(ROOT_DIR, f) for f in res.stdout.split('\0') if f]


def test():
    branch_commit = find_branch_point("origin/master")
    files = diff_files(branch_commit)
    print(files)
    rc, errors = run_ruff(files, fix=True)
    if errors:
        print(errors)
    else:
        print("No lint errors found.")
    assert rc == 0


if __name__ == '__main__':
    conftest.run_this_test(__file__)
