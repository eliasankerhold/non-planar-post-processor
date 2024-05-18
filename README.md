<a name="readme-top"></a>

<div align="center">
  <h3 align="center">Non-Planar Post Processor</h3>

  <p align="center">
    A Python tool to post process 3D-printer G-Code for FDM printing on non-planar surfaces.
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
    <li>
      <a href="#printer-modifications">Printer Modifications</a>
      <ul>
         <li><a href="#print-head">Print Head</a></li>
         <li><a href="#control-interface">Control Interface</a></li>
         <li><a href="#print-bed">Print Bed</a></li>
      </ul>
    </li>
    <li>
      <a href="#setup-and-calibration">Setup and Calibration</a>
      <ul>
         <li><a href="#z-homing-position">Z-Homing Position</a></li>
         <li><a href="#buffer-zones-and-safety-margins">Buffer Zones and Safety Margins</a></li>
         <li><a href="#print-bed-alignment">Print Bed Alignment</a></li>
      </ul>
    </li>
   <li><a href="#general-notes">General Notes</a></li>
    <li><a href="#license">License</a></li>
   <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

<figure align="center">
    <img src="/img/printing.gif" width=600 alt="Non-planar printing">
</figure>

<!-- ABOUT THE PROJECT -->

## About The Project

Non-planar FDM printing is a powerful, yet challenging and vastly complicated topic. This tool is a minimal approach to
enable simple non-planar printing on print beds with sufficiently small curvature. Under the hood, the given gcode is
simply projected onto the bed surface. This projection does not preserve angles and distances and can therefore deform
the original, planar model.

Once hardware and software are properly prepared and set up, the workflow is very simple:

1. Slice the planar model using your preferred standard slicing software (Cura, Slic3r, PrusaSlicer, ...)
2. Post process the gcode output using this tool, which will only alter the toolpaths and leave all other settings
   untouched.
3. Check the non-planar gcode in a preview software and print it.

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

The most straightforward way of using this tool is through its command line interface, accessible through running the
following command in the root directory:

```shell
python nonplanar.py -h
```

This will show an extensive help message with all required info and arguments. To standardize the workflow, command line
arguments can also be saved in a text file and then passed:

```shell
python nonplanar.py @your_arguments.txt
```

Alternatively, the tool can be directly used in any Python code by importing the necessary class:

```python
from nonplanarpp.gcode_handling import GCodeProjector
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- PRINTER MODIFICATIONS -->
## Printer Modifications

<!-- PRINT HEAD -->
### Print Head

<figure align="center">
    <img src="/img/print_head_mods.jpg" width=600 alt="Print head with modifications">
    <figcaption>Highly modified print head with custom nozzle of increased length and "pointyness", adapted mount for the part cooling fan and a movable mounting for the z homing sensor that can be flipped upwards after homing. </figcaption>
</figure>

#### Nozzle

To keep up with increasing print speeds even in entry-level printers, print heads tend to carry beefy part coolers and
are becoming bulkier and bulkier. This drastically limits the clearance between nozzle tip and lower face of the print
head, essentially making non-planar printing impossible without an adapted nozzle.

A nozzle for non-planar printin needs to be:

1. Significantly longer than standard, providing enough clearance between nozzle tip and print head
2. 'pointier' than standard, allowing a higher tolerance for extruding on surfaces that are not orthogonal to the
   nozzle.

For some printer models, such nozzles are commercially available. In the scope of this project however, the nozzle was
custom-made from a brass cylinder.

#### Part Cooling Fan

As mentioned above, the clearance between print head and nozzle tip plays a decisive role in non-planar printing. If a
part cooling fan is required, the standard mounting position is most probably too close to the nozzle tip and should be
changed to a custom solution.

#### Z-Offset Sensor

If the print head houses a sensor that probes the bed in the z homing procedure, chances are high that it also needs to
be adapted. This is not as straightforward as remounting a part cooling fan, since the sensor itself is still required
to home the z axis before starting a print. Here, this issue was solved by designing a rotatable mount that can rotate
the sensor out of the way once the z homing is completed.

<!-- CONTROL INTERFACE -->
### Control Interface

The setup and calibration procedure requires direct control over the printer, meaning the ability to send G-Code
commands and receive responses in a terminal-like environment. Thus, the standard user interface is not sufficient and a
more direct control option has to be set up. In this project, a RasberryPi running OctoPrint was used. If supported by
the printer, a Klipper interface is probably an even more suitable choice.

<!-- PRINT BED -->
### Print Bed

#### Temporary approach

For prototyping and testing, the non-planar print bed can be printed using conventional methods and then serves as a
base for the non-planar structures on top. PLA and soft TPU (for example, 82A) have rather bad adhesion and can be
separated with relative ease. First tests of this project were done by first printing a TPU structure and then using
non-planar printing to extrude PLA onto it. If being extremely careful, it might be possible to use the same TPU base
twice, but effectively this approach only gives single-use print beds.

#### Long-term solution

Once the optimal shape and size of the non-planar bed has been found, the temporary TPU beds should be replaced by a
permanent solution. Aluminium is the obvious choice here, as it is easy to machine, readily and cheaply available and -
most importantly - a great thermal conductor. The print bed design should include a designated area to home the z axis
as well as precise alignment markers.

<figure align="center">
    <img src="/img/print_bed.jpg" width=600 alt="Non-planar print bed">
    <figcaption>Non-planar print bed, CNC machined from an aluminium block with alignment pins and designated z homing areas on the left and right. The print bed is clamped into place by two laser-cut acrylic sheets which are screwed into the build plate. A total of eight print slots are available, here covered with painter's tape to improve print bed adhesion. </figcaption>
</figure>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- SETUP AND CALIBRATION -->

## Setup and Calibration

The following steps only have to be done when setting up a non-planar printing system for the first time or after moving
the system, making changes to the hardware, etc. However, it is utmost important to calibrate the system as precisely as
possible, since any inaccuracies in these steps will limit the printing performance fundamentally.

<!-- Z-HOMING POSITION -->
### Z-Homing Position

Since the non-planar print bed is not uniform, the z axis cannot be homed at any arbitrary position on the bed, but only
in a designated area. However, the x-y position of the z homing procedure are usually not customizable in the printer
firmware. Therefore, an adapted homing procedure has to be prepared and used:

1. Using the standard settings, the z axis has to be homed ("G28 Z").
2. Without moving the print head after z homing, the current position should be queried ("M114") and saved in the
   printer configuration .ini file (example in `testing/ender3v3se_4x2_bed.ini`).
3. Noting that the nozzle position is offset relative to the z sensor position, the print head has to be moved to the
   desired position of z homing.
4. The new z homing coordinates should be saved in the config file mentioned above.

The post processor will replace the "G28" homing command with a two-step procedure, which home x and y axes first. Then,
a coordinate shift is applied to the printer in such a way that the standard z homing position in the original
coordinate system is mapped onto the new position. After homing the z axis, this coordinate shift is reversed and the
printer moves in its standard coordinates.

<!-- BUFFER ZONES AND SAFETY MARGINS -->
### Buffer Zones and Safety Margins

Since points saved in the G-Code instructions for the printer only describe the position of the nozzle tip and don't
take the actual physical dimensions of the nozzle into account. Therefore, sharp edges and sudden changes in print bed
height pose an extreme risk of crashing the nozzle into the bed. The easiest way to prevent this is to feed a slightly
adapted 3D model of the print bed to the post-processing tool. After truthfully modeling the print bed itself, safety
zones should be added as simple blocks around sharp edges or deep valleys. This will make the print head follow the
surface of the adapted model and prevent the crashes.

<figure align="center">
    <img src="/img/model_comparison.jpg" width=600 alt="3D model of non-planar print bed">
    <figcaption>Comparison between actual model of the print bed (lower right) and adapted version with safety and buffer zones (upper left). Using the adapted print bed model in the post-processing procedure prevents the nozzle from crashing at sharp edges.</figcaption>
</figure>

<!-- PRINT BED ALIGNMENT -->
### Print Bed Alignment

There is no actual direct feedback mechanism between printer and the non-planar bed. Basically, the printer does not "
see" the print bed and blindly follows G-Code commands. Therefore, it is absolutely essential that the virtual 3D model
of the print bed agrees with its real-world counterpart in shape, size and position. Since the custom print bed has to
be CNC machined anyway, having a 3D model of the print bed should not be any concern. The remaining part is to position
this virtual print bed at the exact same position (relative to the printer coordiante system) as the real-world one.
This alignment procedure consists of several steps:

1. The print bed is mounted at the desired position and should be secured tightly into place - it shall not be moved at
   all afterward.
2. The x and y axes of the printer have to be homed (NOT the z axis!), e.g. for Marlin firmware "G28 X Y".
3. The nozzle tip has to be moved to align as precisely as possible with the designated alignment point on the print
   bed (in this case a pointy aluminium tip).
4. Read the position of the alignment point in the printer coordinate system (e.g. "M114").
5. The virtual print bed has to be moved such that the alignment point is positioned exactly as on the actual printer.
   Saving the model as an STL file preservers these coordinates since STL is using an absolute coordinate system.
6. Knowing the position of the alignment point in the printer coordinate system as well as the offset between alignment
   point and print area(s) allows to calculate where the models have to placed in order to be printed at the correct
   position. This position in the printer coordinate system might have to be converted further into the coordinate
   system used by the conventional slicer (e.g. Cura using an origin-at-center coordinate system).

<figure align="center">
    <img src="/img/x_alignment.jpg" width=600 alt="Alignment of non-planar print bed">
    <figcaption>Alignment procedure of the non-planar bed on the x-axis. The nozzle is moved to align with the calibration pin as precisely as possible. </figcaption>
</figure>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GENERAL NOTES -->

## General Notes

### Thermal stress

The curved geometry of the different layers seems to increase stress caused by layers cooling down and shrinking. This makes the print more prone to warping and adhesion issues. For PLA prints, it was possible to mitigate these issues by using a high bed temperature (80 C), no part cooling and operating at the lower end of the nozzle temperature range (200 C). 

### Stutter Movements

It can happen that the print head's smooth continuous movements start to stutter and abruptly stop-and-go. Even though this effect has been observed in multiple printers, it is unclear where exactly it comes from. One strong suspicion is that the read rate of the SD card is not fast enough to read a large amount of points at relatively high speeds. This does not really affect the printer durin standard, planar operation. However, since non-planar movements require a lot of interpolated points with small step sizes, the stuttering occurs. One way to mitigate or even fully eliminate this issue is by using a system like OctoPrint which is able to read G-Code from an HDD or SSD and send it to the printer line by line. In the case of OctoPrint, this means choosing "upload locally" instead of saving GCode to the printer's SD card. 

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->

## License

Distributed under the GNU GPLv3 License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!--ACKNOWLEDGEMENTS -->

## Acknowledgements

The project was part of the <a href="https://pdp.fi/">PDP program</a> of the  <a href="https://designfactory.aalto.fi">
Design Factory</a> at <a href="https://www.aalto.fi/en">Aalto University</a> in Helsinki.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->

## Contact

Elias Ankerhold - elias.ankerhold[at]aalto.fi <br>
Jonas Tjepkema - jonas.tjepkema[at]aalto.fi

<p align="right">(<a href="#readme-top">back to top</a>)</p>
