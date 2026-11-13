机器人学的定义
----------------------

本节说明机器人学的定义、历史、关键组成部分与应用。

什么是机器人学？
^^^^^^^^^^^^^^^^^^

机器人是能够自动或半自主执行任务的机器。
机器人学是研究机器人的学科。

“robot” 一词来自捷克语 “robota”，意思是 “强制劳动” 或 “苦工”。
它最早出现在捷克作家 `Karel Čapek`_ 于 1920 年创作的科幻戏剧 `R.U.R.`_\ （Rossum’s Universal Robots）中。
在剧中，机器人是被制造来为人类服务的人造工人，但它们最终反叛了。

随着时间推移，“robot” 开始指能够执行任务的机器或自动化系统，
通常具有一定程度的智能或自主性。

目前，全世界有 200 万台机器人在工作，并且数量每年都在增加。
在机器人应用特别迅速的韩国，
每 1000 人中有 50 台机器人在运行。

.. _`R.U.R.`: https://thereader.mitpress.mit.edu/origin-word-robot-rur/
.. _`Karel Čapek`: https://en.wikipedia.org/wiki/Karel_%C4%8Capek

机器人的历史
^^^^^^^^^^^^^^^^^^^^^^^^^^^

这条时间线突出机器人历史中的关键里程碑：

古代与早期概念（16 世纪之前）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**自动机器**\ 的想法已经存在了数千年。
古代文明设想过机械生物：

- **古希腊（公元前 4 世纪）**——希腊工程师 `Hero of Alexandria`_ 设计了早期的\ **自动机**\ （automata，自行运转的机器），由水或空气驱动。
- **中国与阿拉伯自动机（9–13 世纪）**——像 `Ismail Al-Jazari`_ 这样的发明家制造了精巧的机械装置，包括水钟以及由水力驱动、会走动的自动孔雀。

.. _`Hero of Alexandria`: https://en.wikipedia.org/wiki/Hero_of_Alexandria
.. _`Ismail Al-Jazari`: https://en.wikipedia.org/wiki/Ismail_al-Jazari

现代机器人学的诞生（16 世纪–19 世纪）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- `Leonardo da Vinci’s Robot`_\ （1495）——设计了一个能够机械运动的人形骑士。
- `Jacques de Vaucanson’s Digesting Duck`_\ （1738）——制造了机械装置，例如一只能够“吃”和“消化”的机械鸭。
- `工业革命 <Industrial Revolution_>`_\ （18–19 世纪）——机器开始在工厂中取代人类劳动，为自动化奠定了基础。

.. _`Leonardo da Vinci’s Robot`: https://en.wikipedia.org/wiki/Leonardo%27s_robot
.. _`Jacques de Vaucanson’s Digesting Duck`: https://en.wikipedia.org/wiki/Jacques_de_Vaucanson
.. _`Industrial Revolution`: https://en.wikipedia.org/wiki/Industrial_Revolution

工业机器人的兴起（20 世纪–20 世纪 50 年代）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **“机器人”一词（1921）**——捷克作家 `Karel Čapek`_ 在其戏剧 `R.U.R.`_\ （Rossum’s Universal Robots）中引入了\ *“robot”*\ 一词。
- **早期控制论（20 世纪 40 年代–50 年代）**——`Norbert Wiener`_ 等科学家发展了自调节机器的理论，影响了现代机器人学（控制论，Cybernetics）。

.. _`Norbert Wiener`: https://en.wikipedia.org/wiki/Norbert_Wiener

现代机器人学的诞生（20 世纪 50 年代–80 年代）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **第一台工业机器人（1961）**——由 `George Devol`_ 和 `Joseph Engelberger`_ 创造的 `Unimate`_，是工厂中使用的第一台可编程机器人。
- **人工智能与自主机器人的兴起（20 世纪 70 年代–80 年代）**——研究人员开发了 `Shakey`_\ （斯坦福，1966）等移动机器人以及基于人工智能的控制系统。

.. _`Unimate`: https://en.wikipedia.org/wiki/Unimate
.. _`George Devol`: https://en.wikipedia.org/wiki/George_Devol
.. _`Joseph Engelberger`: https://en.wikipedia.org/wiki/Joseph_Engelberger
.. _`Shakey`: https://en.wikipedia.org/wiki/Shakey_the_robot

先进机器人学与人工智能的融合（20 世纪 90 年代至今）
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **工业机器人**——`Baxter`_ 和 `Amazon Robotics`_ 等先进机器人革新了制造与物流。
- **自动驾驶车辆**——用于机器人出租车的自动驾驶汽车，如 `Waymo`_，以及矿业中的自动运输系统，如 `AHS`_，变得更加先进并达到可商用程度。
- **探测机器人**——用于行星探测（例如 NASA 的 `火星车 <Mars rovers_>`_\）。
- **医疗机器人**——`da Vinci Surgical System`_ 等机器人革新了医疗保健。
- **个人机器人**——`Roomba`_\ （吸尘机器人）和 `Sophia`_\ （人工智能人形机器人）等设备变得流行。
- **服务机器人**——餐厅和酒店中的辅助机器人，例如送餐机器人 `Bellabot`_。
- **协作机器人（无人机）**——无人机表演和配送服务中的协作机器人，如 UAV（无人驾驶飞行器）。

.. _`Baxter`: https://en.wikipedia.org/wiki/Baxter_(robot)
.. _`Amazon Robotics`: https://en.wikipedia.org/wiki/Amazon_Robotics
.. _`Mars rovers`: https://en.wikipedia.org/wiki/Mars_rover
.. _`Waymo`: https://waymo.com/
.. _`AHS`: https://www.futurebridge.com/industry/perspectives-industrial-manufacturing/autonomous-haulage-systems-the-future-of-mining-operations/
.. _`da Vinci Surgical System`: https://en.wikipedia.org/wiki/Da_Vinci_Surgical_System
.. _`Roomba`: https://en.wikipedia.org/wiki/Roomba
.. _`Sophia`: https://en.wikipedia.org/wiki/Sophia_(robot)
.. _`Bellabot`: https://www.pudurobotics.com/en

机器人学的关键组成部分
^^^^^^^^^^^^^^^^^^^^^^^^^^^

机器人学由若干基本组成部分构成：

#. 传感器——从环境采集信息（例如相机、激光雷达、GNSS、陀螺仪、加速度计、轮式编码器）。
#. 执行器——实现运动以及与外界的交互（例如电机、液压系统）。
#. 计算机——处理传感器数据并做出决策（例如微控制器、CPU、GPU）。
#. 电源——提供驱动机器人运行的能量（例如电池、太阳能）。
#. 软件与算法——使机器人能够运行并做出智能决策（例如 ROS、机器学习模型、定位、建图、路径规划、控制）。

本项目 `PythonRobotics` 聚焦于机器人学中的软件与算法部分。
如果你对传感器硬件感兴趣，可以查看 :ref:`机器人的内部传感器 <Internal Sensors for Robots>` 或 :ref:`机器人的外部传感器 <External Sensors for Robots>`。
