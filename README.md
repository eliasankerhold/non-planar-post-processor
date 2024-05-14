<a name="readme-top"></a>

<div align="center">
  <h3 align="center">Non-Planar Post Processor</h3>

  <p align="center">
    A Python tool to post process 3d-printer gcode for printing on non-planar surfaces.
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#installation">Installation</a></li>
        <li><a href="#basic-usage">Basic Usage</a></li>
      </ul>
    </li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

Non-planar FDM printing is a powerful, yet challenging and vastly complicated topic. This tool is a minimal approach to enable simple non-planar printing on print beds with sufficiently small curvature. Under the hood, the given gcode is simply projected onto the bed surface. This projection does not preserve angles and distances and can therefore deform the original, planar model.

From the user-side, the workflow is very simple:
1. Generate an STL file of the modified, curved print bed on which you want to print. 
2. Slice the planar model using your preferred standard slicing software (Cura, Slic3r, PrusaSlicer, ...)
3. Post process the gcode output using this tool, which will only alter the toolpaths and leave all other settings untouched.
4. Check the non-planar gcode in a preview software and print it.

<span style='color:orangered'>
<h3>WARNING!</h3>
    <strong>Be careful when printing, this approach is highly experimental and can cause crashes, issues with bed leveling and other unexpected behavior. Check the result in a gcode previewer before printing.</strong>
</span>
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

The package can be installed through ``pip``.
<!-- INSTALLATION -->
### Installation

To install the package, navigate to its root folder, i.e. where ``pyproject.toml`` is located, and install it using pip:
   ```shell
   python -m pip install .
   ```

<!-- BASIC USAGE -->
### Basic Usage

The most straightforward way of using this tool is through its command line interface, accessible through running the following command in the root directory: 
```shell
python nonplanar.py -h
```

This will show an extensive help message with all required info and arguments. To standardize the workflow, command line arguments can also be saved in a text file and then passed:
```shell
python nonplanar.py @your_arguments.txt
```

Alternatively, the tool can be directly used in any Python code by importing the necessary class:
```python
from nonplanarpp.gcode_handling import GCodeProjector
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LICENSE -->
## License

Distributed under the GNU GPLv3 License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Elias Ankerhold - elias.ankerhold[at].aalto.fi

<p align="right">(<a href="#readme-top">back to top</a>)</p>