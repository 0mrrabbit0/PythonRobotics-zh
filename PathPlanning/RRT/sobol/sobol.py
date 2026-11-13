"""
  许可:
    本代码在 MIT 许可下分发。

  作者:
    i4_sobol 的原始 FORTRAN77 版本由 Bennett Fox 编写。
    MATLAB 版本由 John Burkardt 编写。
    PYTHON 版本由 Corrado Chisari 编写。

    is_prime 的原始 Python 版本由 Corrado Chisari 编写。

    其他函数的原始 MATLAB 版本由 John Burkardt 编写。
    PYTHON 版本由 Corrado Chisari 编写。

    原始代码见
    https://people.sc.fsu.edu/~jburkardt/py_src/sobol/sobol.html

    注: 前缀 i4 表示该函数接受一个数值参数，或
          返回一个在函数内部被解释为 4
          字节整数的数
    注: 前缀 r4 表示该函数接受一个数值参数，或
          返回一个在函数内部被解释为 4
          字节浮点数的数
"""
import math
import sys
import numpy as np

atmost = None
dim_max = None
dim_num_save = None
initialized = None
lastq = None
log_max = None
maxcol = None
poly = None
recipd = None
seed_save = None
v = None


def i4_bit_hi1(n):
    """
     I4_BIT_HI1 返回 I4 中以 2 为底的最高位 1 的位置。

      讨论:

        I4 是一种整数（kind = 4）值。

      Example:

           N    Binary    Hi 1
        ----    --------  ----
           0           0     0
           1           1     1
           2          10     2
           3          11     2
           4         100     3
           5         101     3
           6         110     3
           7         111     3
           8        1000     4
           9        1001     4
          10        1010     4
          11        1011     4
          12        1100     4
          13        1101     4
          14        1110     4
          15        1111     4
          16       10000     5
          17       10001     5
        1023  1111111111    10
        1024 10000000000    11
        1025 10000000001    11

      许可:

        本代码在 GNU LGPL 许可下分发。

      修改:

        26 October 2014

      作者:

        John Burkardt

      Parameters:

        输入，整数 N，待测量的整数。
        N 应为非负。若 N 为非正，该函数
        将始终为 0。

        输出，整数 BIT，最高位的位置。

    """
    i = n
    bit = 0

    while True:

        if i <= 0:
            break

        bit = bit + 1
        i = i // 2

    return bit


def i4_bit_lo0(n):
    """
     I4_BIT_LO0 返回 I4 中以 2 为底的最低位 0 的位置。

      讨论:

        I4 是一种整数（kind = 4）值。

      Example:

           N    Binary    Lo 0
        ----    --------  ----
           0           0     1
           1           1     2
           2          10     1
           3          11     3
           4         100     1
           5         101     2
           6         110     1
           7         111     4
           8        1000     1
           9        1001     2
          10        1010     1
          11        1011     3
          12        1100     1
          13        1101     2
          14        1110     1
          15        1111     5
          16       10000     1
          17       10001     2
        1023  1111111111    11
        1024 10000000000     1
        1025 10000000001     2

      许可:

        本代码在 GNU LGPL 许可下分发。

      修改:

        08 February 2018

      作者:

        John Burkardt

      Parameters:

        输入，整数 N，待测量的整数。
        N 应为非负。

        输出，整数 BIT，最低位 1 的位置。

    """
    bit = 0
    i = n

    while True:

        bit = bit + 1
        i2 = i // 2

        if i == 2 * i2:
            break

        i = i2

    return bit


def i4_sobol_generate(m, n, skip):
    """


     I4_SOBOL_GENERATE 生成一个 Sobol 数据集。

      许可:

        本代码在 MIT 许可下分发。

      修改:

        22 February 2011

      作者:

        原始 MATLAB 版本由 John Burkardt 编写。
        PYTHON 版本由 Corrado Chisari 编写。

      Parameters:

        输入，整数 M，空间维数。

        输入，整数 N，要生成的点数。

        输入，整数 SKIP，要跳过的初始点数。

        输出，实数 R(M,N)，这些点。

    """
    r = np.zeros((m, n))
    for j in range(1, n + 1):
        seed = skip + j - 2
        [r[0:m, j - 1], seed] = i4_sobol(m, seed)
    return r


def i4_sobol(dim_num, seed):
    """


     I4_SOBOL 每次调用生成一个新的 Sobol 拟随机向量。

      讨论:

        本程序采用了 Antonov 与 Saleev 的思想。

      许可:

        本代码在 MIT 许可下分发。

      修改:

        22 February 2011

      作者:

        原始 FORTRAN77 版本由 Bennett Fox 编写。
        MATLAB 版本由 John Burkardt 编写。
        PYTHON 版本由 Corrado Chisari 编写。

      Reference:

        Antonov, Saleev,
        USSR Computational Mathematics and Mathematical Physics,
        第 19 卷，19，第 252 - 256 页。

        Paul Bratley, Bennett Fox,
        算法 659:
        Sobol 拟随机序列生成器的实现，
        ACM Transactions on Mathematical Software,
        第 14 卷，第 1 期，第 88-100 页，1988 年。

        Bennett Fox,
        算法 647:
        拟随机序列生成器的实现
        及其相对效率，
        ACM Transactions on Mathematical Software,
        第 12 卷，第 4 期，第 362-376 页，1986 年。

        Ilya Sobol,
        USSR Computational Mathematics and Mathematical Physics,
        第 16 卷，第 236-242 页，1977 年。

        Ilya Sobol, Levitan,
        多维立方体内均匀分布点的生成
        （俄文），
        Preprint IPM Akad. Nauk SSSR,
        第 40 号，Moscow 1976。

      Parameters:

        输入，整数 DIM_NUM，空间维数。
        DIM_NUM 必须满足 1 <= DIM_NUM <= 40。

        输入/输出，整数 SEED，序列的“种子”。
        它实质上是待生成拟随机
        值在序列中的索引。    输出时，SEED 已被设为
        合适的下一个值，通常就是 SEED+1。
        若输入的 SEED 小于 0，则按 0 处理。
        输入值 0 请求序列的第一个（第 0 个）元素。

        输出，实数 QUASI(DIM_NUM)，下一个拟随机向量。

    """

    global atmost
    global dim_max
    global dim_num_save
    global initialized
    global lastq
    global log_max
    global maxcol
    global poly
    global recipd
    global seed_save
    global v

    if not initialized or dim_num != dim_num_save:
        initialized = 1
        dim_max = 40
        dim_num_save = -1
        log_max = 30
        seed_save = -1
        #
        #    初始化 V（的一部分）。
        #
        v = np.zeros((dim_max, log_max))
        v[0:40, 0] = np.transpose([
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
        ])

        v[2:40, 1] = np.transpose([
            1, 3, 1, 3, 1, 3, 3, 1, 3, 1, 3, 1, 3, 1, 1, 3, 1, 3, 1, 3, 1, 3,
            3, 1, 3, 1, 3, 1, 3, 1, 1, 3, 1, 3, 1, 3, 1, 3
        ])

        v[3:40, 2] = np.transpose([
            7, 5, 1, 3, 3, 7, 5, 5, 7, 7, 1, 3, 3, 7, 5, 1, 1, 5, 3, 3, 1, 7,
            5, 1, 3, 3, 7, 5, 1, 1, 5, 7, 7, 5, 1, 3, 3
        ])

        v[5:40, 3] = np.transpose([
            1, 7, 9, 13, 11, 1, 3, 7, 9, 5, 13, 13, 11, 3, 15, 5, 3, 15, 7, 9,
            13, 9, 1, 11, 7, 5, 15, 1, 15, 11, 5, 3, 1, 7, 9
        ])

        v[7:40, 4] = np.transpose([
            9, 3, 27, 15, 29, 21, 23, 19, 11, 25, 7, 13, 17, 1, 25, 29, 3, 31,
            11, 5, 23, 27, 19, 21, 5, 1, 17, 13, 7, 15, 9, 31, 9
        ])

        v[13:40, 5] = np.transpose([
            37, 33, 7, 5, 11, 39, 63, 27, 17, 15, 23, 29, 3, 21, 13, 31, 25, 9,
            49, 33, 19, 29, 11, 19, 27, 15, 25
        ])

        v[19:40, 6] = np.transpose([
            13, 33, 115, 41, 79, 17, 29, 119, 75, 73, 105, 7, 59, 65, 21, 3,
            113, 61, 89, 45, 107
        ])

        v[37:40, 7] = np.transpose([7, 23, 39])
        #
        #    设置 POLY。
        #
        poly = [
            1, 3, 7, 11, 13, 19, 25, 37, 59, 47, 61, 55, 41, 67, 97, 91, 109,
            103, 115, 131, 193, 137, 145, 143, 241, 157, 185, 167, 229, 171,
            213, 191, 253, 203, 211, 239, 247, 285, 369, 299
        ]

        atmost = 2**log_max - 1
        #
        #    求 ATMOST 中的位数。
        #
        maxcol = i4_bit_hi1(atmost)
        #
        #    初始化 V 的第 1 行。
        #
        v[0, 0:maxcol] = 1

        # 仅当维度发生变化时才需要做的事。

    if dim_num != dim_num_save:
        #
        #    检查参数。
        #
        if (dim_num < 1 or dim_max < dim_num):
            print('I4_SOBOL - Fatal error!')
            print('    The spatial dimension DIM_NUM should satisfy:')
            print(f'        1 <= DIM_NUM <= {dim_max:d}')
            print(f'    But this input value is DIM_NUM = {dim_num:d}')
            return None

        dim_num_save = dim_num
        #
        #    初始化 V 的其余各行。
        #
        for i in range(2, dim_num + 1):
            #
            #    整数 POLY(I) 的各个位给出多项式
            #    I 的形式。
            #
            #    由二进制编码求多项式 I 的次数。
            #
            j = poly[i - 1]
            m = 0
            while True:
                j = math.floor(j / 2.)
                if (j <= 0):
                    break
                m = m + 1
            #
            #    展开该位模式，以分离逻辑
            #    数组 INCLUD 的各个分量。
            #
            j = poly[i - 1]
            includ = np.zeros(m)
            for k in range(m, 0, -1):
                j2 = math.floor(j / 2.)
                includ[k - 1] = (j != 2 * j2)
                j = j2
            #
            #    按 Bratley 与 Fox 第 2 节所述，计算
            #    第 I 行的其余元素。
            #
            for j in range(m + 1, maxcol + 1):
                newv = v[i - 1, j - m - 1]
                l_var = 1
                for k in range(1, m + 1):
                    l_var = 2 * l_var
                    if (includ[k - 1]):
                        newv = np.bitwise_xor(
                            int(newv), int(l_var * v[i - 1, j - k - 1]))
                v[i - 1, j - 1] = newv
#
#    将 V 的各列乘以相应的 2 的幂。
#
        l_var = 1
        for j in range(maxcol - 1, 0, -1):
            l_var = 2 * l_var
            v[0:dim_num, j - 1] = v[0:dim_num, j - 1] * l_var
#
#    RECIPD 等于 1/（V 中元素的公分母）。
#
        recipd = 1.0 / (2 * l_var)
        lastq = np.zeros(dim_num)

    seed = int(math.floor(seed))

    if (seed < 0):
        seed = 0

    if (seed == 0):
        l_var = 1
        lastq = np.zeros(dim_num)

    elif (seed == seed_save + 1):
        #
        #    找出 SEED 中最右侧 0 的位置。
        #
        l_var = i4_bit_lo0(seed)

    elif seed <= seed_save:

        seed_save = 0
        lastq = np.zeros(dim_num)

        for seed_temp in range(int(seed_save), int(seed)):
            l_var = i4_bit_lo0(seed_temp)
            for i in range(1, dim_num + 1):
                lastq[i - 1] = np.bitwise_xor(
                    int(lastq[i - 1]), int(v[i - 1, l_var - 1]))

        l_var = i4_bit_lo0(seed)

    elif (seed_save + 1 < seed):

        for seed_temp in range(int(seed_save + 1), int(seed)):
            l_var = i4_bit_lo0(seed_temp)
            for i in range(1, dim_num + 1):
                lastq[i - 1] = np.bitwise_xor(
                    int(lastq[i - 1]), int(v[i - 1, l_var - 1]))

        l_var = i4_bit_lo0(seed)
#
#    检查用户的调用次数是否过多！
#
    if maxcol < l_var:
        print('I4_SOBOL - Fatal error!')
        print('    Too many calls!')
        print(f'    MAXCOL = {maxcol:d}\n')
        print(f'    L =            {l_var:d}\n')
        return None


#
#    计算 QUASI 的新分量。
#
    quasi = np.zeros(dim_num)
    for i in range(1, dim_num + 1):
        quasi[i - 1] = lastq[i - 1] * recipd
        lastq[i - 1] = np.bitwise_xor(
            int(lastq[i - 1]), int(v[i - 1, l_var - 1]))

    seed_save = seed
    seed = seed + 1

    return [quasi, seed]


def i4_uniform_ab(a, b, seed):
    """


     I4_UNIFORM_AB 返回一个缩放后的伪随机 I4。

      讨论:

        该伪随机数将被缩放，使其均匀分布
        在 A 与 B 之间。

      许可:

        本代码在 GNU LGPL 许可下分发。

      修改:

        05 April 2013

      作者:

        John Burkardt

      Reference:

        Paul Bratley, Bennett Fox, Linus Schrage,
        《仿真指南》，
        第二版，
        Springer, 1987,
        ISBN: 0387964673,
        LC: QA76.9.C65.B73.

        Bennett Fox,
        算法 647:
        拟随机序列生成器的实现
        及其相对效率，
        ACM Transactions on Mathematical Software,
        第 12 卷，第 4 期，1986 年 12 月，第 362-376 页。

        Pierre L'Ecuyer,
        《随机数生成》，
        载于《仿真手册》，
        Jerry Banks 编，
        Wiley, 1998,
        ISBN: 0471134031,
        LC: T57.62.H37.

        Peter Lewis, Allen Goodman, James Miller,
        System/360 的伪随机数生成器，
        IBM Systems Journal,
        第 8 卷，第 2 期，1969 年，第 136-143 页。

      Parameters:

        输入，整数 A、B，可接受的最小值与最大值。

        输入，整数 SEED，随机数生成器的种子。

        输出，整数 C，随机选取的整数。

        输出，整数 SEED，更新后的种子。

    """

    i4_huge = 2147483647

    seed = int(seed)

    seed = (seed % i4_huge)

    if seed < 0:
        seed = seed + i4_huge

    if seed == 0:
        print('')
        print('I4_UNIFORM_AB - Fatal error!')
        print('  Input SEED = 0!')
        sys.exit('I4_UNIFORM_AB - Fatal error!')

    k = (seed // 127773)

    seed = 167 * (seed - k * 127773) - k * 2836

    if seed < 0:
        seed = seed + i4_huge

    r = seed * 4.656612875E-10
    #
    #  将 R 缩放到介于 A-0.5 与 B+0.5 之间。
    #
    a = round(a)
    b = round(b)

    r = (1.0 - r) * (min(a, b) - 0.5) \
        + r * (max(a, b) + 0.5)
    #
    #  通过舍入将 R 转换为介于 A 与 B 之间的整数。
    #
    value = round(r)

    value = max(value, min(a, b))
    value = min(value, max(a, b))
    value = int(value)

    return value, seed


def prime_ge(n):
    """


     PRIME_GE 返回大于或等于 N 的最小素数。

      Example:

          N    PRIME_GE

        -10     2
          1     2
          2     2
          3     3
          4     5
          5     5
          6     7
          7     7
          8    11
          9    11
         10    11

      许可:

        本代码在 MIT 许可下分发。

      修改:

        22 February 2011

      作者:

        原始 MATLAB 版本由 John Burkardt 编写。
        PYTHON 版本由 Corrado Chisari 编写。

      Parameters:

        输入，整数 N，待定界的数。

        输出，整数 P，大于
        或等于 N 的最小素数。

    """
    p = max(math.ceil(n), 2)
    while not isprime(p):
        p = p + 1

    return p


def isprime(n):
    """


     IS_PRIME 在 N 为素数时返回 True，否则返回 False

      许可:

        本代码在 MIT 许可下分发。

      修改:

        22 February 2011

      作者:

        Corrado Chisari

      Parameters:

        输入，整数 N，待检查的数。

        输出，布尔值，True 或 False

    """
    if n != int(n) or n < 1:
        return False
    p = 2
    while p < n:
        if n % p == 0:
            return False
        p += 1

    return True


def r4_uniform_01(seed):
    """


     R4_UNIFORM_01 返回一个单位区间上的伪随机 R4。

      讨论:

        本程序实现如下递推

          seed = 167 * seed mod ( 2^31 - 1 )
          r = seed / ( 2^31 - 1 )

        整数运算所需位数从不超过 32 位，
        其中包括一个符号位。

        若初始种子为 12345，则前三次计算为

          Input     Output      R4_UNIFORM_01
          SEED      SEED

             12345   207482415  0.096616
         207482415  1790989824  0.833995
        1790989824  2035175616  0.947702

      许可:

        本代码在 GNU LGPL 许可下分发。

      修改:

        04 April 2013

      作者:

        John Burkardt

      Reference:

        Paul Bratley, Bennett Fox, Linus Schrage,
        《仿真指南》，
        第二版，
        Springer, 1987,
        ISBN: 0387964673,
        LC: QA76.9.C65.B73.

        Bennett Fox,
        算法 647:
        拟随机序列生成器的实现
        及其相对效率，
        ACM Transactions on Mathematical Software,
        第 12 卷，第 4 期，1986 年 12 月，第 362-376 页。

        Pierre L'Ecuyer,
        《随机数生成》，
        载于《仿真手册》，
        Jerry Banks 编，
        Wiley, 1998,
        ISBN: 0471134031,
        LC: T57.62.H37.

        Peter Lewis, Allen Goodman, James Miller,
        System/360 的伪随机数生成器，
        IBM Systems Journal,
        第 8 卷，第 2 期，1969 年，第 136-143 页。

      Parameters:

        输入，整数 SEED，用于生成
        输出随机数的整数“种子”。  SEED 不得为 0。

        输出，实数 R，介于 0 与 1 之间的随机值。

        输出，整数 SEED，更新后的种子。  该值
        通常会作为下一次调用的输入种子。

    """

    i4_huge = 2147483647

    if (seed == 0):
        print('')
        print('R4_UNIFORM_01 - Fatal error!')
        print('  Input SEED = 0!')
        sys.exit('R4_UNIFORM_01 - Fatal error!')

    seed = (seed % i4_huge)

    if seed < 0:
        seed = seed + i4_huge

    k = (seed // 127773)

    seed = 167 * (seed - k * 127773) - k * 2836

    if seed < 0:
        seed = seed + i4_huge

    r = seed * 4.656612875E-10

    return r, seed


def r8mat_write(filename, m, n, a):
    """


     R8MAT_WRITE 将一个 R8MAT 写入文件。

      许可:

        本代码在 GNU LGPL 许可下分发。

      修改:

        12 October 2014

      作者:

        John Burkardt

      Parameters:

        输入，字符串 FILENAME，输出文件名。

        输入，整数 M，A 的行数。

        输入，整数 N，A 的列数。

        输入，实数 A(M,N)，该矩阵。
    """

    with open(filename, 'w') as output:
        for i in range(0, m):
            for j in range(0, n):
                s = f'  {a[i, j]:g}'
                output.write(s)
            output.write('\n')


def tau_sobol(dim_num):
    """


     TAU_SOBOL 定义 Sobol 序列的有利起始种子。

      讨论:

        对于空间维数 1 到 13，本程序返回
        一个“有利”的 TAU 值，据此可以确定
        Sobol 序列中合适的起始点。

        这些起始点具有 N = 2**K 的形式，其中
        对于积分问题，希望满足
                TAU + DIM_NUM - 1 <= K
        而对于优化问题，希望满足
                TAU < K.

      许可:

        本代码在 MIT 许可下分发。

      修改:

        22 February 2011

      作者:

        原始 FORTRAN77 版本由 Bennett Fox 编写。
        MATLAB 版本由 John Burkardt 编写。
        PYTHON 版本由 Corrado Chisari 编写。

      Reference:

        IA Antonov, VM Saleev,
        USSR Computational Mathematics and Mathematical Physics,
        第 19 卷，19，第 252 - 256 页。

        Paul Bratley, Bennett Fox,
        算法 659:
        Sobol 拟随机序列生成器的实现，
        ACM Transactions on Mathematical Software,
        第 14 卷，第 1 期，第 88-100 页，1988 年。

        Bennett Fox,
        算法 647:
        拟随机序列生成器的实现
        及其相对效率，
        ACM Transactions on Mathematical Software,
        第 12 卷，第 4 期，第 362-376 页，1986 年。

        Stephen Joe, Frances Kuo
        关于算法 659 的注记:
        Sobol 拟随机序列生成器的实现，
        ACM Transactions on Mathematical Software,
        第 29 卷，第 1 期，第 49-57 页，2003 年 3 月。

        Ilya Sobol,
        USSR Computational Mathematics and Mathematical Physics,
        第 16 卷，第 236-242 页，1977 年。

        Ilya Sobol, YL Levitan,
        多维立方体内均匀分布点的生成
        （俄文），
        Preprint IPM Akad. Nauk SSSR,
        第 40 号，Moscow 1976。

      Parameters:

                输入，整数 DIM_NUM，空间维数。    仅当取值
                为 1 到 13 时才会得到有用的结果。

                输出，整数 TAU，即 TAU 的值。

    """
    dim_max = 13

    tau_table = [0, 0, 1, 3, 5, 8, 11, 15, 19, 23, 27, 31, 35]

    if 1 <= dim_num <= dim_max:
        tau = tau_table[dim_num]
    else:
        tau = -1

    return tau
