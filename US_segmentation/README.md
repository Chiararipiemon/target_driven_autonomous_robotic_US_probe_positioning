On the simulated ultrasound volumes, a pretrained neural network https://github.com/luohwu/UltraBones100k was tested for automatic bone segmentation without task-specific fine-tuning. While some vertebral structures were detected, the masks were often noisy, incomplete, and inconsistent across probe poses, leading to unreliable partial reconstructions. 
**References**:
@article{WU2025110435,
title = {UltraBones100k: A reliable automated labeling method and large-scale dataset for ultrasound-based bone surface extraction},
journal = {Computers in Biology and Medicine},
volume = {194},
pages = {110435},
year = {2025},
issn = {0010-4825},
doi = {https://doi.org/10.1016/j.compbiomed.2025.110435},
url = {https://www.sciencedirect.com/science/article/pii/S0010482525007863},
author = {Luohong Wu and Nicola A. Cavalcanti and Matthias Seibold and Giuseppe Loggia and Lisa Reissner and Jonas Hein and Silvan Beeler and Arnd Viehöfer and Stephan Wirth and Lilian Calvet and Philipp Fürnstahl},
keywords = {Ultrasound bone segmentation, Bone surface segmentation, Bone surface reconstruction, Ultrasound image analysis, Computer-assisted orthopedic surgery},
}

The main goal of this component is to obain from the US data a coarse bone reconstruction.
