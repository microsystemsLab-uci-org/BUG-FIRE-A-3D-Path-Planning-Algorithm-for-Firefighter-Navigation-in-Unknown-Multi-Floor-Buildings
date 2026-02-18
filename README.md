# 3D BUG Algorithm for Multi-Floor Path Planning
This repository contains the MATLAB code for the paper "BUG-FIRE: A 3D Path Planning Algorithm with Augmented Reality Integration for Firefighter Navigation in Unknown Multi-Floor Buildings," published in the Fire Technology Journal. You can find the paper [here]().

## Abstract
Path planning in unknown, multi-floor buildings presents significant challenges for both autonomous systems and human navigation. Navigating such environments requires reaching a target location without prior knowledge of the building’s layout or obstacles. In particular, firefighters face these challenges during emergencies, where low-visibility conditions and restricted movement complicate navigation. This paper introduces BUG-FIRE, a 3D path planning algorithm designed to assist firefighter navigation in single-tower buildings without prior environmental knowledge. BUG-FIRE combines BUG2, an algorithm widely used in autonomous robotic systems, and adapts it for first responder navigation, using it for intra-floor movement. For inter-floor transitions, we integrate an emergency exit exploration algorithm that ensures goal convergence while complying with building code standards, such as mandatory stairwell connectivity and emergency signage. We validate BUG-FIRE through 250 Monte Carlo simulations in a virtual building model, demonstrating a 100\% success rate within the tested simulation scenarios for reaching randomly selected targets across multiple floors. The results show that agents with a 30-meter vision radius reduce path length by 28.2\ compared to those with only 0.5-meter visibility, highlighting the importance of environmental perception. To bridge the gap to real-world applications, we implement a YOLOv8-based vision system on the Magic Leap 2 AR headset, enabling real-time emergency signage recognition and dynamic AR guidance. Experimental results indicate that users can successfully navigate 3D paths utilizing AR guidance, with trajectory lengths closely matching simulated benchmarks.

## Concept
This repository presents a novel 3D BUG algorithm designed for autonomous navigation in unknown multi-floor indoor environments, with a specific focus on assisting firefighters. The algorithm addresses the unique challenges of three-dimensional path planning in complex building structures where prior map information is unavailable. This implementation guarantees convergence for pedestrian agents or robots navigating to complex goals within multi-story buildings. While the resulting paths may not be optimal due to the absence of prior map information, the algorithm ensures that a viable path to the target is always found, regardless of the building's layout complexity.

## BUG-FIRE Simulations - Usage Requirements

* Clone this repository
* Download Peter's Coke [repository](https://petercorke.com/toolboxes/robotics-toolbox/). The 2D BUG algorithm function and some navigation functions are used from this library.
```
git clone https://github.com/petercorke/robotics-toolbox-matlab rtb
git clone https://github.com/petercorke/spatial-math smtb
git clone https://github.com/petercorke/toolbox-common-matlab commo
```

## BUG-FIRE Implementation on an AR device

### Emergency Exit Signs Recognition

#### Dataset

The emergency exit sign dataset used to train the YOLOv8 model for the Magic Leap 2 implementation is publicly available on Roboflow:

🔗 https://app.roboflow.com/emergency-exit-signs/emergency-exit-signs-v2/models/emergency-exit-signs-v2/8

The dataset includes six classes of exit signs:
- Right  
- Left  
- Left/Right  
- Straight  
- Backwards  
- Straight/Backwards  

It also contains synthetic augmentations (occlusions, lighting variations, blur, rotations, etc.) to emulate degraded visibility conditions typical in fire scenarios.

#### ML Training

### Navigation Environment Implementation into Unity

## How to Cite
If you use this work, please cite the following paper:
> **BUG-FIRE: A 3D Path Planning Algorithm with Augmented Reality Integration for Firefighter Navigation in Unknown Multi-Floor Buildings**<br>
> E. Sangenis, and A. M. Shkel<br>
> *Fire Technology* (Springer Nature), vol.62, no.42, 2026.
> doi: 10.1007/s10694-026-01872-9

**BibTeX:**

```bibtex
@ARTICLE{bugFireSangenis,
  author = {Sangenis, Eudald and Andrei M. Shkel},
  journal = {Fire Technology},
  publisher = {Springer Nature},
  title = {BUG-FIRE: A 3D Path Planning Algorithm with Augmented Reality Integration for Firefighter Navigation in Unknown Multi-Floor Buildings}, 
  year = {2026},
  volume = {62},
  number = {41},
  url = {https://doi.org/10.1007/s10694-026-01872-9}
}
```

## Authorship
Eudald Sangenis (esangeni@uci.edu)
