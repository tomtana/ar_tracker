# ar_tracker

## Introduction

The rospackage ar_tracker is a realtime fiducial marker tracker based on ARToolkit 5. 

What is this package for?
- Reliably tracking a single fiducial maker
- very high framerates: tested on an 2048x1080 image with 35fps at 10% CPU usage
- being suitable for outdoor environments with changing illumination conditions due to the adaptive thresholding of ART5
- detecting multiple markers and selecting one out of them for high speed tracking

How is this achieved?
- The algorithm scales the image such that the marker is of constant size in the image e.g. 40 pixel
- Then a roi is defined around the marker
- Each step a new roi position is predicted with the gradient computed out of the marker pose

## Installation

>- Clone project 
- download ARToolkit 5: https://github.com/artoolkit/artoolkit5
- compile ARToolkit 5 by following the instructions
- create a folder "ARToolkit" in the package root
- copy the folders "lib" and "include" from the artoolkit5 library to the "ARToolkit" folder you just created
- catkin make
- check out the config folder
