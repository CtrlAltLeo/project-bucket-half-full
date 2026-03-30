# What is PyLauncher?

PyLauncher is a __Virtual Environment__ that allows us to use some of the libraries required by OpenCV. A VE is like a mini-installation of a specific version of python. In our case, it is version 3.11. This allows us to install and run dependencies that were deprecated starting in version 3.12 and later. It also gets around the problem of pip not installing anything if it detects a package manager like apt. In order to run any program in the OpenCV folder, you will likely need to first enter the VE using PyLauncher.

## How Do I Enter and Exit PyLauncher?

First, make sure you have Python 3.11.14 installed, then, from the project root, run:
``source pyLauncher/bin/activate``
You should then see your command line gain the prefix "(myenv) which is the internal name of the VE. If you run ``python --version``, you should see it return version 3.11.14. To leave the environment, simply type ``deactivate`` Since we already used this environment, all the dependencies should be already installed and ready to go!

## How do I make my own Virtual Environment?

If you're having problems with the existing PyLauncher or just want the learning experience, here's what you'll need to do: first install Python 3.11. Remember, it needs to be that specific version. Now, assuming it installed under the package name python3.11, run ``python3.11 -m venv [environment name]``, which will create a blank canvas for you. Then, you can install ``pip install -r OpenCV/requirements.txt``

You should now be good to go!
