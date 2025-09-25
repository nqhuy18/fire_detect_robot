# Vision System
This branch contains the code for a vision system that utilizes a camera to detect early signs of a fire in a supermarket

## Installation
To install the necessary dependencies, run the following command:
```bash
   conda create --name vision_system python=3.10
   conda activate vision_system
   pip install ultralytics
```

## Data Structure
The data is organized in the following structure:
```bash
├── dataset.yaml
├── images/
│   ├── train/
│   │   ├── image1.jpg
│   │   └── image2.jpg
│   └── val/
│       ├── image3.jpg
│       └── image4.jpg
└── labels/
    ├── train/
    │   ├── image1.txt
    │   └── image2.txt
    └── val/
        ├── image3.txt
        └── image4.txt
```

## Run on your custom dataset
To train the YOLOv11 model on your custom dataset, use the following command and instruction:

Step 1: Get your dataset labelled using robotflow with YOLO format, then download zip files 
to your local computer.

Step 2: Run the following command below to extract the images and structure images and labels 
following to the section Data Structure.

```bash
   python extract_data.py
```

## Todo List
- [x] Set up the environment
- [x] Install necessary libraries
- [x] Collect data
- [x] Train data on YOLOv11
- [x] Initial evaluation
- [ ] Fine-tune model
- [ ] Build physics-prior model
- [ ] Integrate sensors' information for the robot's actions
- [ ] Deploy model on the robot
