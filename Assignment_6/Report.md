OS: macOS Big Sur 11.2.1 Compiler: Apple clang version 12.0.5 (clang-1205.0.22.9)
The code compiles and runs successfully on my MacBook Pro (M1).

------
### Compiling the Project
```mkdir build```\
```cd build; cmake -DCMAKE_BUILD_TYPE=Release ..```\
```make; ./RasterViewer```

-------
### Ex.1, 2, 3, 4, 5, 6: The program works as expected. 
<kbd>i</kbd>: enable triangle insertion mode\
<kbd>o</kbd>: enable triangle translation mode\
<kbd>p</kbd>: enable triangle deletion mode\
<kbd>h</kbd>: the selected triangle will rotate by 10 degree counter-clockwise\
<kbd>j</kbd>: the selected triangle will rotate by 10 degree clockwise\
<kbd>k</kbd>: the selected triangle will be scaled up by 25%\
<kbd>l</kbd>: the selected triangle will be scaled down by 25%\
<kbd>c</kbd>: enable triangle color mode\
<kbd>1</kbd>-<kbd>9</kbd>: the selected vertex will change its color\
<kbd>=</kbd>: increase the zoom by 20% zooming in in the center of the screen.\
<kbd>-</kbd>: decrease the zoom by 20% zooming out in the center of the screen.\
<kbd>w</kbd>: pan the view by 20% of the visible part of the scene, down by 20% of the window size.\
<kbd>a</kbd>: pan the view by 20% of the visible part of the scene, right by 20% of the window size.\
<kbd>s</kbd>: pan the view by 20% of the visible part of the scene, up by 20% of the window size.\
<kbd>d</kbd>: pan the view by 20% of the visible part of the scene, left by 20% of the window size.\
<kbd>n</kbd>: create a new frame of your animation\
<kbd>m</kbd>: play the animation\
<kbd>q</kbd>: clear the frames of the animation

--------------------------
### Ex.6: Implemented the Linear Interpolation.