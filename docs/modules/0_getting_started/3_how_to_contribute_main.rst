如何贡献
=================

本文档说明如何为本项目做贡献。
可以通过以下几种方式为本项目做贡献：

#. `添加新的算法示例 <Adding a new algorithm example_>`_
#. `报告并修复缺陷 <Reporting and fixing a defect_>`_
#. `为已有示例补充缺失的文档 <Adding missed documentations for existing examples_>`_
#. `支持本项目 <Supporting this project_>`_

贡献之前
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

贡献之前请检查以下事项：

理解本项目
---------------------------

请阅读 :ref:`What is PythonRobotics?` 一节以及这篇论文
`PythonRobotics: a Python code collection of robotics algorithms`_，
以理解本项目的理念。

.. _`PythonRobotics: a Python code collection of robotics algorithms`: https://arxiv.org/abs/1808.10703

.. _`Check your Python version.`:

检查你的 Python 版本
---------------------------

我们只接受面向 Python 3.13.x 或更高版本的 PR。

我们不接受面向 Python 2.x 的 PR。

.. _`Before opening a pull request`:

打开拉取请求之前
--------------------------------

打开 PR 之前，请检查适用于你这次更改的事项。下面的步骤会更详细地说明这些要点。

- **范围：** 聚焦于一个算法、一个缺陷或一个文档主题。把大的更改拆成较小的 PR，并先在 Issue 中提议重大更改。
- **差异：** 使用单独的分支，并对照 ``master`` 检查 diff。去掉无关的格式、参数、依赖和 CI 配置更改。
- **可读性：** 使用描述性的名称和简短清晰的函数。即使在做优化时，也要让代码容易与算法的方程对应起来。
- **兼容性：** 遵循 `检查你的 Python 版本 <Check your Python version._>`_ 以及当前的 CI 配置。只使用已有的 :ref:`Requirements` 库。
- **既有行为：** 除非更改是有意的并已加以说明，否则保留默认参数和可运行的示例。不要为了掩盖失败而削弱测试。
- **测试：** 对于代码更改，在 ``tests`` 下添加或扩展测试，检查期望结果和相关边界情况。以无动画方式运行示例，并在本地运行 ``bash runtests.sh``，包括其中的代码风格检查。
- **文档：** 更新已有的官方 RST 页面，或者为新示例添加一页。在那里解释算法和数学；在文件头和文档字符串中解释代码、输入、输出和单位。这些说明用英文撰写。
- **导航与渲染：** 把新页面加入相应父页面的 ``toctree``。构建文档并检查生成的 HTML，包括方程、列表、链接和图像。
- **动画：** 对于新示例或改变了视觉行为的更改，确认动画能让人看懂结果、颜色和线条。把新的 GIF 提交到 `PythonRoboticsGifs`_，并在文档中链接它们。
- **PR 描述：** 准备 Issue 链接、更改原因和测试结果。在有帮助时附上更改前后的结果；对于性能改进，附上测试配置以及可比较的计时。

.. _`Adding a new algorithm example`:

1. 添加新的算法示例
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

这是添加新算法示例的分步说明。

步骤 1：选择要实现的算法
-----------------------------------------

在选择算法之前，请阅读 :ref:`getting started` 文档，
以理解本项目的理念并配置你的开发环境。

如果某个算法被广泛使用并且是成功的，请创建 Issue，向我们的社区提议它。

如果有人通过点赞或发表正面评论表示同意，就进入下一步。

只创建 Issue 来提议添加某种算法也是可以的，也许会有人去实现它。

在这种情况下，请分享任何可用于实现的论文或文档。


步骤 2：用基于 matplotlib 的动画实现算法
----------------------------------------------------------------

实现算法时，请牢记以下几点。

1. 只使用 Python。不接受其他语言的代码。

2. 使用 `检查你的 Python 版本 <Check your Python version._>`_ 中说明的 Python 版本，并用当前 CI 配置所使用的版本运行示例。

3. 使用基于 matplotlib 的动画展示算法如何工作。在适用时把起点、目标、障碍物和结果标清楚，并解释颜色和线条的含义。动画应帮助读者理解算法的行为，例如检测到障碍物之后路径如何变化。

4. 只使用当前的 :ref:`Requirements` 库，不要添加新的依赖。

5. 保持代码简单、易于跟随。主要目标是帮助初学者理解算法。优先考虑可读性，而不是性能或额外的抽象，并让实现能够从算法的方程中辨认出来。避免只对默认仿真有效的捷径。

6. 使用描述性名称，而不是未加解释的缩写或单字母名称。例如，用 ``parent_index`` 而不是 ``pind``。如果数学符号有助于把代码和方程对应起来，请解释它们的含义。把过长的函数和复杂条件拆成命名清晰的函数或中间变量，并遵循邻近示例的风格。

7. 添加简短的文件头，说明该示例、其作者和相关参考文献。使用文档字符串解释函数、类、输入、输出和配置参数，在适用时包括单位。重构时保留已有的有用说明。

8. 使用具名配置参数，而不是未加解释的数值常量。除非改变默认值是预期修复的一部分，否则保留已有默认值；针对某一次测试的参数应设置在该测试内部。

9. 把可运行的示例放在 ``main()`` 函数中，并用 ``show_animation`` 保护绘图，使示例在没有动画时也能运行。当已有的 Python、NumPy、SciPy 和项目函数能让算法更易读时，优先使用它们，而不是重复同一套实现。


步骤 3：添加单元测试
----------------------
如果你添加了新的算法示例代码，请在 `tests 目录 <tests dir_>`_ 下添加单元测试文件。

这段示例测试代码也许对你有帮助：`test_a_star.py`_。

至少要在单元测试中尝试以无动画方式运行示例代码。

扩展已有示例时，尽可能把测试用例加到它已有的测试文件中。给测试起能说明场景的描述性名称，例如 ``test_no_obstacle`` 或 ``test_too_big_step_size``。覆盖新增的算法变体和相关边界情况。

对于缺陷修复，要复现所报告的问题，并断言期望结果，而不仅仅是代码运行时不抛出异常。保持已有测试和默认示例可以工作。不要只是为了让改过的算法通过而削弱断言；对任何有意的行为变化都要加以说明并测试。

从仓库根目录在本地运行测试套件：

.. code-block:: bash

   bash runtests.sh

`test_codestyle.py`_ 会检查你的 PR 代码的代码风格。


.. _`how to write doc`:

步骤 4：撰写关于该算法的文档
----------------------------------------------
请描述算法的基本步骤、数学背景和相关参考文献，并配上图和动画 GIF。在有助于读者理解示例时，说明它与相关算法的区别以及它在何处有用。

本项目使用 `Sphinx`_ 作为文档构建工具，所有文档都用 `reStructuredText`_ 编写。

对于已有示例，更新其官方文档，而不是另写一份 README 或重复的说明。对于新示例，在 `文档模块目录 <doc modules dir_>`_ 中相应类别下添加 ``*_main.rst`` 页面，并把它列入父页面的 ``toctree``。遵循邻近页面的命名和 toctree 条目，使新页面出现在文档导航中。

请查看其他文档了解细节。

你可以依据 `文档 README <doc README_>`_ 在本地构建文档。

构建之后，打开生成的 HTML 或该 PR 的文档产物，并检查改过的页面。确认方程、列表、链接、图像和动画都能正确渲染，并且可以从其父页面到达该页。仅构建成功并不能保证页面看起来正确。

要创建 GIF 动画，可以使用这个工具：`matplotrecorder`_。

创建的 GIF 文件应存放在 `PythonRoboticsGifs`_ 仓库中，因此请创建 PR 来添加它，并在文档中引用它。

请注意，基于 `reStructuredText`_ 的文档应只聚焦于该示例的数学和算法。

与代码相关的说明应放在 Python 脚本中，作为脚本的头部注释或各函数的文档字符串。

此外，每份文档都应有指向 GitHub 上代码的链接。你可以使用 Sphinx 的 `autodoc`_ 模块，通过 `.. autoclass::`、`.. autofunction::` 和 `.. automodule` 轻松添加该链接。

使用这个 `autodoc`_ 模块后，生成的文档会带有指向 GitHub 上代码的链接，如下所示：

.. image:: /_static/img/source_link_1.png

点击该链接后，会跳转到 GitHub 上的源代码，如下所示：

.. image:: /_static/img/source_link_2.png



.. _`submit a pull request`:

步骤 5：提交拉取请求，并根据评审修改代码
------------------------------------------------------------

先使用 `打开拉取请求之前 <Before opening a pull request_>`_ 清单，然后在代码、测试和文档都准备好时提交你的 PR。

在 PR 描述中，链接相关 Issue，并说明问题是什么、为什么这项更改能解决它，以及你如何测试。对于仿真的更改，在有帮助时附上更改前后的结果或动画。对于性能改进，说明所用技术，并提供可比较的计时以及获得这些计时所用的配置。

首先，请在代码评审之前修复所有 CI 错误。

检查测试、代码风格和文档构建的结果。如果某个失败看起来与你的更改无关，请报告失败的检查和相关日志，而不是修改 CI 设置或删掉测试来掩盖失败。

你可以从 CI 面板查看你的 PR 文档。

在 “ci/circleci: build_doc” CI 成功之后，
你可以点击 “ci/circleci: build_doc artifact” CI 的 [Details] 来访问你的 PR 文档。

.. image:: /_static/img/doc_ci.png

此后，我将开始评审。

请注意，这是我的业余项目；感谢你在评审过程中的耐心。

　

.. _`Reporting and fixing a defect`:

2. 报告并修复缺陷
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

报告并修复缺陷同样是很好的贡献。

报告 Issue 时，请提供以下信息：

- 清晰简洁地描述缺陷是什么。
- 清晰简洁地描述你预期会发生什么。
- 如适用，附上有助于说明问题的屏幕截图。
- 操作系统版本
- Python 版本
- 各个库的版本

如果你想修复某个缺陷，可以在 `带有 bug 标签的 Issue <bug labeled issues_>`_ 中查找已报告的问题。

如果你修复了已有代码中的缺陷，请在测试代码中添加测试函数，以表明该问题已经解决。
把会失败的输入用作回归用例，并断言修正后的行为，包括相关边界条件。把无关的更改放在单独的 PR 中。

这份 `提交拉取请求 <submit a pull request_>`_ 的文档对提交拉取请求会有帮助。


.. _`Adding missed documentations for existing examples`:

3. 为已有示例补充缺失的文档
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

为已有示例补充缺失的文档同样是很好的贡献。

如果你查看 `Python Robotics 文档 <Python Robotics Docs_>`_，会注意到有些示例只有仿真 GIF、简短的概述，或者只写着 TBD.，而没有详细的算法或数学描述。这些文档需要改进。

这份 `如何撰写文档 <how to write doc_>`_ 的说明对撰写文档会有帮助。

.. _`Supporting this project`:

4. 支持本项目
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

在资金上支持本项目同样是很好的贡献！！

如果你或你的公司希望支持本项目，请考虑：

- `在 GitHub Sponsors 上赞助 @AtsushiSakai <Sponsor @AtsushiSakai on GitHub Sponsors_>`_

- `在 Patreon 上成为支持者或赞助者 <Become a backer or sponsor on Patreon_>`_

- `通过 PayPal 一次性捐赠 <One-time donation via PayPal_>`_

如果你希望以其他方式支持我们，请创建 Issue 与我们联系。

当前主要赞助者：

#. `GitHub`_：他们为本开源软件的开发提供 GitHub Copilot Pro 许可。
#. `JetBrains`_：他们为本开源软件的开发提供其 IDE 的免费许可。
#. `1Password`_：他们为本开源项目提供 1Password 团队许可的免费授权。



.. _`Python Robotics Docs`: https://atsushisakai.github.io/PythonRobotics
.. _`bug labeled issues`: https://github.com/AtsushiSakai/PythonRobotics/issues?q=is%3Aissue+is%3Aopen+label%3Abug
.. _`tests dir`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/tests
.. _`test_a_star.py`: https://github.com/AtsushiSakai/PythonRobotics/blob/master/tests/test_a_star.py
.. _`Sphinx`: https://www.sphinx-doc.org/
.. _`reStructuredText`: https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html
.. _`doc modules dir`: https://github.com/AtsushiSakai/PythonRobotics/tree/master/docs/modules
.. _`doc README`: https://github.com/AtsushiSakai/PythonRobotics/blob/master/docs/README.md
.. _`test_codestyle.py`: https://github.com/AtsushiSakai/PythonRobotics/blob/master/tests/test_codestyle.py
.. _`JetBrains`: https://www.jetbrains.com/
.. _`GitHub`: https://www.github.com/
.. _`Sponsor @AtsushiSakai on GitHub Sponsors`: https://github.com/sponsors/AtsushiSakai
.. _`Become a backer or sponsor on Patreon`: https://www.patreon.com/myenigma
.. _`One-time donation via PayPal`: https://www.paypal.com/paypalme/myenigmapay/
.. _`1Password`: https://github.com/1Password/for-open-source
.. _`matplotrecorder`: https://github.com/AtsushiSakai/matplotrecorder
.. _`PythonRoboticsGifs`: https://github.com/AtsushiSakai/PythonRoboticsGifs
.. _`autodoc`: https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html
