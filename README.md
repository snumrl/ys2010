# ys2010

How to run
==================


in linux,
-------------------

1. sudo apt-get install python-dev python-pip python-numpy libgle3 freeglut3-dev libfreetype6-dev libpng12-dev
2. install [boost](http://boost.org)
3. sudo apt-get install libfltk1.3-dev python-fltk libode-dev 
4. pip install pyopengl pyopengl-accelerate pyode matplotlib
5. build implicitMassSpringSolver2 and VirtualPhysics2010 in PyCommon/external_libraries( just make )
6. copy .a files to PyCommon/modules/usr/lib (if path doesn't exist, make dir)
7. "python setup.py build"  in PyCommon/modules
8. run main_PreprocessMcfg.py, main_PreprocessSeg.py, and main_PreprocessBvh.py (adjust the files on your case)
9. python main_TrackingExamples2.py
