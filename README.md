# Spacial_Mapping_System

## Table of contents

- [Overview](#overview)
  - [The challenge](#the-challenge)
  - [Screenshot](#screenshot)
- [My Process](#my-Process)
  - [Built with](#built-with)
  - [Code Snippets](#code-Snippets)
  - [What I learned](#what-i-learned)

## Overview

### The challenge
- Design and build an embedded spatial measurement system using a time-of-flight sensor to acquire
information about the area around you. Using a rotary mechanism to provide a 360 degree measurement
of distance within a single vertical geometric plane (e.g., y-z). 

- Mapped spatial information is stored in onboard memory and later communicated to a personal computer or web application for reconstruction and graphical presentation.

### Screenshot

#### Spacial Mapping Device
Consists of MSP432E401Y Microcontroller,  VL53L1X Time of Flight (ToF) sensor, Stepper Motor, and Active Low Push Button
![Device](./images/Device.jpg)

#### Hallway Reconstruction
Using Python OpenGL library to reconstruct data

![HallwayReconstruction](./images/HallwayReconstruction.png)

## My process

### Built with

- Python: OpenGL
- C:
- Assembly

### Code snippets


