Polygon Walker
===========
The polygon walker splits an n-sided polygon into x many equal areas along a specified starting line.

How to run
---------------
./walker --vehicles[-v] ... --polygon[-p] ... --start[-s] ... --error[-e] ... demo

--vehicles[-v] n specifies n many areas
--polygon[-p] x0 y0 x1 y1 ... specifies the number of points that will be given followed by the points
--start[-s] x0 y0 x1 y1 specifies the two starting points
--error[-e] e specifies e error (must be between 0 and 1)
demo starts the demo with default test cases
