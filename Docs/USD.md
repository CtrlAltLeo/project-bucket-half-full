# Working with Universal Scene Description
Since the foundation of this project relies on means of simulation, familiarity with construction 3-D objects is essential.

Developing and combining the bucket asset with a material to fill it with is performed within Isaac Sim (Working with this reference application is gone into more detail in the [StartHere](Docs/StartHere.md) and [IsaacSim](Docs/IsaacSim.md) files. However, it was not ideal to jump straight into this process without understanding the basics of working with 3D objects. This includes the code to build objects, the environments such objects are subject to, and the different applications to manipulate the objects.

Our simulation workflow consisted of 3 main parts, gone into further detail below:
1. Working soley within the usdview application and coding objects.
2. Using Blender to simulate more detailed assets while developing objects with a GUI.
3. Shifting work into Isaac Sim and implementing assets supplied by Bobcat.

The first two parts of the workflow are relatively acheivable using your personal PC. usdview is a very light application and, although Blender is a heavier application, can be adjusted to render more smoothly based on your hardware. We have been running these two applications on Windows PCs.  We recommend downloading these two applications (links to do so are presented in the following sections). Isaac Sim can be setup with the help of the Bobcat team.

*One quirky issue we have run into: if you happen to be running an ARM architecture, you may come across some issues downloading and installing usdview. It is best to use a common x86 processor instead.*

## usdview
Using usdview is our first step to creating objects and getting an "introduction" to working directly with the USD file format (.usd or .usda).

We can begin as simple as developing a basic object, such as a sphere or cube, move into adjusting their size and placement, and finally, adjust their material properties such as color and roughness. Below is an example of how we created a single red sphere.

    #usda 1.0
    (
        defaultPrim = "hello"
    )
    
    def Material "TestMaterial1"
    {
        token outputs:surface.connect      = </TestMaterial1/pbrMat1.outputs:surface>
        
        def Shader "pbrMat1"
        {
            uniform token info:id = "UsdPreviewSurface"        
            token outputs:surface
    
            # Material Inputs
            int inputs:useSpecularWorkflow       = 1
    	color3f  inputs:diffuseColor         = (0.5, 0.5, 0.5)
            color3f  inputs:specularColor        = (0.8, 0.8, 0.8)
            color3f  inputs:emissiveColor        = (1, 0, 0)
            float    inputs:roughness            = 0.3
        }
    }
     
    def Xform "hello1"
    {
        custom double3 xformOp:translate = (2, 2, 2)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    
        def Sphere "Sphere1"
        {
            float3[] extent = [(-2, -2, -2), (2, 2, 2)]
            color3f[] primvars:displayColor = [(1, 0, 0)]
            double radius = 1
    	rel material:binding = </TestMaterial1>
        }
    }
This file is then uploaded into usdview and gives us access to observing our sphere in a 3D environment. We are able to easily move, zoom, and rotate our camera to see our objects from any angle.

To begin working with and viewing USD files, usdview download and documentation can be found [here](https://docs.omniverse.nvidia.com/usd/latest/usdview/index.html).

## Blender

### Aside: STL -> USD using Blender
To demonstrate some of the flexibility USD has to offer, we tested converting a STL (.stl) file to a USD (.usda) file and loading this into usdview. This can be done simply by:
- Downloading any STL file of your choosing (or creating your own)
- Opening the file in Blender
- Exporting as a USD file
- Uploading the exported file into usdview 

## Versions & Tooling
*Just in case...*

We used the following versions of the applications:
- Blender: Version **5.0**
- USD: Version **25.08**
- Python: Version **3.12**
