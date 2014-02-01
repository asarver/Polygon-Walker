Polygon Walker
===========
The polygon walker splits an n-sided polygon into x many equal areas along a specified starting line.

How to run
---------------
./walker [options]

Options
---------------

**-s** *n*, **--segments=**_n_       Specifies n many areas.

**-p** *x0 y0 x1 y1 ...*, **--polygon=**_x0 y0 x1 y1 ..._ Specifies the points of the polygon in x y coordinates. The points must be listed in counter clock wise or clock wise order.

**-s** *x0 y0 x1 y1*, **--start=**_x0 y0 x1 y1_ Specifies two points where the polygon will be split. The two points must be on an existing edge.

**-e** *err*, **--error**_err_ Specifies a percentage error of how equal the segments shoudl be. err must be between 0 and 1.

**--demo** Starts the default test cases.

**--help** Displays the help menu. The help menu is also displayed when there are no arguments.
