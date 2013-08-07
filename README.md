box2dweb-fix
============
**box2dweb 手动优化计划**


### 介绍

box2d (<http://www.box2d.org>) 是世界上最知名的开源2D物理引擎, 原版采用C++开发. 有很多人都尝试着将其移植到其他语言环境下. 而javascript的box2d也有若干版本.


其中被使用最广的是 box2dweb (<https://code.google.com/p/box2dweb>). 但是,它并不是人工移植的版本,而是作者利用一个工具将ActionScript3版本的box2d转换成了js版本.

虽然如今机器转换的效果已经非常出色, 但是这个box2dweb仍然存在大量的代码层面的问题, 导致性能受到一定的影响.

而该 **box2dweb-fix** 项目的目的就是建立一个原始box2dweb的**代码层面的优化版本**


================


这个项目非常耗时, 需要一步一步的完善, 欢迎大家参与并报告各种bug。


	注意： 如果你要使用该优化版本， 请直接使用dist下的文件， 不要使用lib下的文件.
	lib下的文件其实是我为了方便优化工作而生成的一些中间文件，并不保证能够正确运行。

================

#### 优化工作的过程

其实很简单, 就是手动修改lib下的文件, 然后执行 

	node make-all-in-one.js

之后就会在 dist目录下 生成 box2d-all-in-one.js ,然后运行测试用例来测试(目前测试用例还不够,测试方式也略麻烦, 后续会慢慢完善).


**优化举例:**

	// box2dweb 源代码里的片段
	
	function b2Contact() {
		b2Contact.b2Contact.apply(this, arguments);
		if (this.constructor === b2Contact)
			this.b2Contact.apply(this, arguments);
	};
	
    b2Contact.b2Contact = function () {

         // 略去一些代码 AAA …

    };

    b2Contact.prototype.b2Contact = function () {

        // 略去一些代码  BBB …

    }

当我们执行

	new b2Contact()
	
时,代码内部绕了一个很大的圈子.其实上面的代码完全可以换成

	function b2Contact() {
		// 略去一些代码 AAA …
		// 略去一些代码  BBB …
	};
	
虽然这未必是性能的瓶颈, 但是这种代码从任何角度看,都是不科学的. box2dweb中还有很多类似的缺点.

该fix项目就是努力的去把这些修正,让box2dweb更优雅一些.

**注意** 以上代码只是举例, 不是所有的这种代码都能直接这么粗暴的修改,因为box2dweb的类继承机制也很绕, 优化时要确保不会破坏子类的行为.

	

================

#### 更进一步的优化

算法和架构上的优化也许才是box2dweb最需要优化的地方, 但是这超出了该fix项目的初衷和我的能力.

目前我正在开发一款更精简、性能更高(相对于box2dweb JPE Chipmunk-js)的javascript版本2D物理引擎. 希望它能够满足追求高性能(但对结果精确度和功能丰富性要求不高)的、从事js开发的朋友.








