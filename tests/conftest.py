"""使测试能够运行的路径处理。"""
import sys
import os
import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(TEST_DIR)  # 以便测试代码导入本文件。
ROOT_DIR = os.path.dirname(TEST_DIR)
sys.path.append(ROOT_DIR)


def run_this_test(file):
    pytest.main(args=["-W", "error", "-Werror", "--pythonwarnings=error", os.path.abspath(file)])
