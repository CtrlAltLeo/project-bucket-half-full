# Working with Universal Scene Description
Since the foundation of this project relies on means of simulation, familiarity with construction 3-D objects is essential.

Developing and combining the bucket asset with a material to fill it with is performed within Isaac Sim (Working with this reference application is gone into more detail in the [StartHere](Docs/StartHere.md) and [IsaacSim](Docs/IsaacSim.md) files. However, it was not ideal to jump straight into this process without understanding the basics of working with 3D objects. This includes the code to build objects, the environments such objects are subject to, and the different applications to manipulate the objects.

Our simulation workflow consisted of 3 main parts, gone into further detail below:
1. Working soley within the usdview application and coding objects.
2. Using Blender to simulate more detailed assets while developing objects with a GUI.
3. Shifting work into Isaac Sim and implementing assets supplied by Bobcat.

## usdview
Using usdview is our first step to creating objects and getting an "introduction" to working directly with the USD file format (.usd or .usda).

We can begin as simple as developing a basic object, such as a sphere or cube, move into adjusting their size and placement, and finally, adjust their material properties such as color and roughness.

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

The usdview download and documentation can be found [here](https://docs.omniverse.nvidia.com/usd/latest/usdview/index.html).
