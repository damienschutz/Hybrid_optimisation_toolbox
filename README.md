# Hybrid_optimisation_toolbox
Optimal control toolbox using spatial algebra and CasADi to generate trajectories for 3D and 2D systems

This toolbox uses the Matlab programming language

Before starting any modelling or optimisations you must run the file **add_path_Hybrid_Optimisation_Toolbox.m**. This includes the CasADi, spatial algebra, the models and the optimisation functions to the Matlab path so that you can start modelling. 

## Setting up an example
Open the **Examples** folder. Here there is an example of a monopod hoppper moving both forward and jumping in place. The files in this folder are commented to explain how to set up an optimisation problem. 
If you are interested in how the model is constructed, go the the **Models** folder and open the one labelled **hopper.m**. This script is also commented and indicates how to set up a model for optimisation. 

Other examples include a horse transverse and a cheetah rotary gallop, although these are a fair bit more advanced. 

## Cheetah and horse data set
In the folder lablelled **Cheetah_v_horse_experiments** you will find the entire dataset that comprised of my Masters project. In here there are four sets of 200 optimisation problem data sets. Each folder contains a particular model, performing a particular gallop. Thereafter this gallop is modelled at four different speeds. Lastly, the spinal compliance and spring stiffness value is parametrised and changed over 50 different values. The resulting trajectory data from all these experiments in kept in the labelled folders. If you wish to investigate this data, you can simply load it into the matlab path and animate it or plot it in a similar fashion to the examples mentionedx above. 
