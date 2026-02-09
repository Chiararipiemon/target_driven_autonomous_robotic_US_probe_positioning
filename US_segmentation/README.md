On the simulated ultrasound volumes, a pretrained neural network https://github.com/luohwu/UltraBones100k was tested for automatic bone segmentation without task-specific fine-tuning. While some vertebral structures were detected, the masks were often noisy, incomplete, and inconsistent across probe poses, leading to unreliable partial reconstructions. 

**References**:
```
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
```
The main goal of this component is to obain from the US data a coarse bone reconstruction.

1. Activate the right conda environment
```
conda activate ultrabones
```
2. Save the US frames as 2D Image set in a folder like this: */home/chiara_piemontese/UltraBones100k/7_test/my_spine_frames_7*
3. Modify the notebook with the correct input folder and output folders.
4. Run the notebook:
```
cd jupyter notebook AI_ultrasound_segmentation/segment_spines_images.ipynb
```
5. Import the masks as 2D Image Set and modify the modality in Label Map

Experiments were run on an NVIDIA GeForce RTX 3090 (24 GB VRAM) using NVIDIA driver 560.35.03 (CUDA 12.6)

<img width="872" height="607" alt="image" src="https://github.com/user-attachments/assets/a6a79d2e-965e-4adf-8ba7-454f032e30a9" />
