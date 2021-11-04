# A Rollover Strategy for Wrist Damage Reduction in a Forward Falling Humanoid

[**Dongdong Liu**](http://mechatronics.engineering.nyu.edu/people/phd-candidates/dongdong-liu.php),  [**Yuhang Lin**](), [**Vikram Kapila**](http://mechatronics.engineering.nyu.edu/people/vikram-kapila)

![Overview](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/1.gif)

|[Abstract](#abstract)|[Code](#code-github)|[Paper](#paper-arxiv)|[Results](#results)|[Acknowledgment](#acknowledgment)|

## Abstract
The last few years have witnessed an increasing interest in the use of humanoid robots for diverse technological applications. Since the center of mass (CoM) of a typical humanoid lies at a relatively high elevation, such a robot often experiences instability during its operation and has a high likelihood of fall. Thus, it is necessary to endow these robots with a robust method to reduce damage that may result from the impact of a fall. Prior research on humans undergoing a forward fall has revealed that the deployment of a rollover strategy along the longitudinal axis can lower the impact force experienced on the hand to effectively reduce wrist injuries. Yet, analogous research for humanoids has received scant attention. To address this research gap, in this work, we consider the optimal design, implementation, and examination of a rollover strategy-similar to the one for humans-for a humanoid robot by employing a differential dynamic programming (DDP) approach. In addition to providing an overview of the theoretical formulation of our methodology, using results from repeated forward falling experiments with a humanoid, we demonstrate that a reliable application of the rollover strategy considerably reduces the impact force vis-à-vis the bimanual fall approach, to protect the wrist of our robot. Moreover, the experiments showcase that the proposed method is also effective in reducing the impact on the torso of the humanoid. With the use of the rollover methodology, a typical humanoid can overcome a key obstacle to its broad adoption and deployment in real-world applications.

## [Code (GitHub)](https://github.com/nyu-legged-group/Rollover/tree/main/)
```
The code is copyrighted by the authors. Permission to copy and use 
 this software for noncommercial use is hereby granted provided: (a)
 this notice is retained in all copies, (2) the publication describing
 the method (indicated below) is clearly cited, and (3) the
 distribution from which the code was obtained is clearly cited. For
 all other uses, please contact the authors.
 
 The software code is provided "as is" with ABSOLUTELY NO WARRANTY
 expressed or implied. Use at your own risk.

This code provides an implementation of the method described in the
following publication: 

Dongdong Liu, Yuhang Lin, and Vikram Kapila    
"A Rollover Strategy for Wrist Damage Reduction in a Forward Falling Humanoid (ICMA-IEEE)". 
``` 

## [Paper (ICMA-IEEE)](https://ieeexplore.ieee.org/abstract/document/9512722)
To cite our paper:
```
@INPROCEEDINGS{9512722,
      title={A Rollover Strategy for Wrist Damage Reduction in a Forward Falling Humanoid},  
      author={Liu, Dongdong and Lin, Yuhang and Kapila, Vikram},
      booktitle={2021 IEEE International Conference on Mechatronics and Automation (ICMA)}, 
      year={2021},
      pages={293-300},
}
```

## Concept 
**The concept of humanoid rollover motion: (a) Initial condition, (b) and (c) falling phase, (d) contact phase, (e) rollover phase, and (f) fallen phase. Blue and green represent left and right limbs, respectively.**
![Concept](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/2.gif)

## Simulation Results
**Initial condition of fall.**
![Simulation_joint_position](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/3.gif)

**Benchmark results for simulated joint positions in rollover fall for (a) upper and (b) lower extremity.**
![Simulation_joint_position](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/4.gif)

**Benchmark simulation results for com trajectory in (a) bimanual and (b) rollover fall.**
![Simulation_CoM](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/5.gif)

**Benchmark simulation timeline snapshots for rollover optimization.**
![Simulation_timeline](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/6.gif)

**Benchmark simulation result for both bimanual and rollover strategies, starting at the same time for (a) impact force, (b) total mechanical energy.**
![Simulation_result](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/7.gif)

## Experiment
**(a) A humanoid with encoder-equipped motors mounted on each joint and inertial measurement unit (imu) mounted on its torso, and (b) the forward fall test bed produced by the DDP method.**
![Experiment_setup](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/8.gif)

**Experimental results for bimanual and rollover fall: (a) Impact force measured by a force sensor and (b) linear acceleration of torso during the experiment as measured by the imu. The measurements for the bimanual and rollover experiments are aligned at their contact time.**
![Experiment_result](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/9.gif)

**Timeline snapshots for a rollover experiment.**
![Experiment_result](https://github.com/nyu-legged-group/Rollover/blob/main/docs/figs/10.gif)

## Acknowledgment
 The research is supported in part by the National Science Foundation under an ITEST grant DRL-1614085, RET Site grant EEC-1542286, and DRK-12 grant DRL-1417769. D. Liu thanks his lab colleagues, particularly P. Chauhan, for helping edit the early drafts of manuscript.