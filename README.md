# Hole Detection for Point Clouds

## Usage

Build the solution with Visual Studio, then run `.\HoleDetection data-set.txt` where `data-set.txt` is your point cloud data, 
one point per line, space separated coordinates.

The program will output the center, radius and normals for all holes found to stdout. It will also create a `boundary.txt` which will 
will contain boundary points for the holes found.

### Visualization

An IPython notebook is included that I use for testing and verification. The `boundary.txt` and the original dataset can be plotted 
to check if the program works correctly.

## References

Bendels, G., R. Schnabel and R. Klein. “Detecting Holes in Point Set Surfaces.” J. WSCG 14 (2006): 89-96.
