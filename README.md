![Agent Based Tectonics](https://raw.githubusercontent.com/Co-de-iT/DesignByData_ATA2020/master/%40%20images/cover_00.jpg)

# Agent-based Tectonics @ DesignByData 2020  
  
Codes used in the Agent-based tectonics workshop held @ Design By Data 18-22 November 2019 - tutor: Alessio Erioli, [Co-de-iT](https://www.co-de-it.com)

Please note that this material is a compendium to the workshop, so many implied instructions, premises and cautions given during the dat-to-day development have not been included.

Tools used: Rhinoceros 3D v6 (includes Grasshopper), Visual Studio 2019.

---

## @ utilities

Contains .gha asssemblies and .dll libraries and general purpose .gh definitions used in the workshop.

**3Dpeople_20190923** - 3D people as meshes in 3 different resolutions

**Util_Clipping plane - Turntable base.3dm**  
**Util-01_clipping plane anim.gh**  
**Util-02_turntable.gh**  
**Util-03_record animation.gh**  
These files are helpers to generate, respectively: an animation of a moving clipping plane (for a model tomography), a turntable of one or more geometries, an animation of the agents from their trajectories as polylines

**Util_post-processing-Dendro** - template for isosurfacing line-base network geometries. Reading the [Dendro](https://www.food4rhino.com/app/dendro) plugin documentation is strongly suggested here
<br>


### @ utilities/Components.zip
**CurlNoise.gha** - calculates Curl Noise for a point in x,y,z - useful to generate spatial vector field data

**FileToScript2.gha** - syncs the code of a C# or VB scripting component in Grasshopper with an external editor - this is an updated version for Rhino6 of FileToScript.gha, a tool written by Mateusz Zwierzycki wrapping up a code by Vicente Soler - additional code to update it for Rhino 6 by Daniel Fink, wrapped and recompiled as a .gha assembly by Alessio Erioli. [Original discussion on FileToScript](https://www.grasshopper3d.com/forum/topics/file-to-script-maths?groupUrl=milkbox&).

**Noises.dll** - library with Simplex and Curl Noise generation functions - see **day 02/CS_02_functions.gh**

**SimplexNoise.dll** - library with Simplex Noise generation functions, it can be used to embed Noise calculations in a custom C# script
<br>

### @ utilities/Display Modes
Contains a bunch of customized Display Modes for Rhino 6 - they can be installed in Rhino from:  
_Tools > Options > View > Display Modes > Import_
<br>

### @ utilities/Mesh Modeling
Rhino files and Grasshopper definitions for basic Mesh modeling (low poly to subdivision techniques)

---
## dayly progress

These folders contain the daily progress of the course. When present, the **in class** folder contains the WIP files developed that day
  
### day 01

**01-00_iterative strategies - intuition.gh** - introduction to iterative strategies in Grasshopper - intuitive approach (standard compopnents + Anemone plug-in)   
**01-01_environment and field - intuition.gh** - reading information from an environment/field - intuitive approach  
**01-02_boundary behaviors intuition.gh** - simple boundary behavior - intuitive approach  
**01-03_environment and field - wrap - intuition.gh** - boundary wrap behavior - intuitive approach


**CS_00_intro.gh** - introduction to C# programming in Grasshopper  
**CS_01_data 01.gh** - data types in C# - part 1  
**CS_01_data 02.gh** - data types in C# - part 2 - loops and conditional statements
  
  
### day 02

**02-00_stigmergy - basic - intuition.gh** - reading and writing information in an environment - intuitive approach

**CS_02_functions.gh** - functions in C#  
**CS_03_classes.gh** - classes and objects in C#
  
  
### day 03

**CS_04_gradient descent.gh** - gradient descent example in C#  
**CS_05_delegates example.gh** - explanation of delegates, anonymous functions and lambda syntax in C#  
**CS_06_RTree point search.gh** - using RTree data structure in C# - simple example of nearest neighbours search


#### day 03/AgentSystem

Visual Studio project folder for the basic Agent System setup

**AgentSystem.gh** - a flocking agent system based on Craig Reynolds classical rules
  
  
### day 04

**Mesh Base treatment.gh + 3dm** - files to build vertex-colored meshes for scalar and vector data fields, to use with the Agent_Bodies workflow.

**Post-Processing_Dendro.gh** - file for isosurfacing line-base network geometries. Reading the [Dendro](https://www.food4rhino.com/app/dendro) plugin documentation is strongly suggested here. Since there was little to no time to show it during the seminar, I have prepared a 30 min definition walkthrough [here](https://youtu.be/tub6E9cFefE). Please watch the video before using the definition.
<br>

### day 04/Agent_Bodies
Visual Studio project folder for the Agent Bodies strategies (for the assigment).

**Agent_Bodies.gh** - the final version of the Agent System with bodies - agents are populating a mesh surface, read scalar and vector data and have simple polyline bodies whose limbs search connection with the neighbours and their global formation results in a performative ornamentation

