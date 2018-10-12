# `Data` Folder
Folder for organizing, storing, and sharing recorded data.

## File descriptions
### [bag_to_csv.py](/bag_to_csv.py)
- Convert ROS bag format file(s) to csv format file(s). Time in rows. All topics in the ROS bag file as columns. Can name a specific file or convert all files in the directory. 

### IMU and GPS on 2018/07/23
- The files are too big for git so the file name in this document is link to the file in google drive for download. You must be using your hawaii.edu alias to access the link.
- Coupled set of ROS bag files, calibration and driving data, taken in my car driving around (from Hawaii Kai to UH I think)
- Data containsData contains information from 2 sparkfun 14001 9dof IMUs and one Adafruit GPS unit using ROS drivers from the the ROS package repository. 

### [calibration_20180723](https://drive.google.com/file/d/11BClcgSuAI-0nzK6ghGmRxgEXhnlAXu7/view?usp=sharing)
- Board sitting on the dash of my car, car running but not moving
  
### [driving_20180723](https://drive.google.com/file/d/1IM72OgZHopVvkrx_QZf1jbMyFT_Ob6vw/view?usp=sharing)
- Board sitting on the dash of my car, car driving

### 2018/02/23 range test `20180223_rangeTestFourGpsOneImu.bag`
- Range test at ??? with multiple GPS units.  Probably two of them emulate the ground station (and therefore sit still in real life) and two of them emulate the boat (which move in real life).

### `2018-09-29-14-38-21_imageRawRemoved.bag`
This bag files is 563 MB, and therefore too large for Git.  It is hosted in our Kanaloa team drive ([link is here](https://drive.google.com/open?id=1p4ZIJCwoiUhmrM-hd2_6z2QyNh6vNqyI)).  This version has all topic except for the IMU and GPS due to some interop errors during the field test.  The imageRaw topics were removed from the bag file to reduce it size (the imageCompressed topics still remain).  [The full version (~15 GB) can be found here](https://drive.google.com/open?id=1SHyPV9j2-9kHUz8eULi5Pyd8TYDwxmKp)
