# Multibounce Glints

Description

This code implements the key ideas of the Siggraph Asia 2020 paper:

"Path Cuts: Efficient Rendering of Pure Specular Light Transport", by

	Beibei Wang, Miloš Hašan, Ling-Qi Yan.

It's under the framework of Mitsuba and a new utils plug-in named mbglints is added. There are two kinds of input for the mesh: the shape from XML, or a height field from the commandline.

Heightfield as input:

mtsutil.exe mbglints xmlFile output.exr heightfield.exr hfScale bounceCount errorThreshold

Mesh as input:
mtsutil.exe mbglints xmlFile output.exr bounceCount errorThreshold

Usage:

The code is under the framework of Mitsuba Renderer 0.5. It's only tested on Windows, so not sure about Mac OS. To compile this code, please use the following steps:

1. Find a dependencies for Mitsuba in the website of Mitsuba renderer, and then make it under the folder mitsuba.

2. Use CMake 2.8 to generate a solution of VS 2013. (We found CMake 3.0+ does not work in our case). 

3. Open the solution, and then build it. 


Example:

We also provided a testing scene with height field as input. With the provided script, the scene could be rendered directly.

The XML can be found [here](scenes/xml/doubleSlabOursReflect.xml), an input normal map can be found [here](scenes/xml/heightfield/isotropic512.exr) and a binary (Windows) can be found [here](scenes/Release). 

The command is also shown here:

mtsutil.exe mbglints xml\doubleSlabOursReflect.xml doubleslabR_ours.exr  xml\heightField\isotropic512.exr 1 1 0.001
