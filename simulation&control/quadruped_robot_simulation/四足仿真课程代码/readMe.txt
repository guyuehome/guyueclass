Make sure you have following files in your directory, in order to run the various examples:

1. remApi.m
2. remoteApiProto.m
3. the appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)
4. simpleTest.m (or any other example program)

-------------------------------------------------------------------

If you choose not to use the prototype file ("remoteApiProto.m"), then you will have to make sure you have a compiler
set-up for Matlab (mex -setup). You will also need "extApi.h" in this folder, and you will have to instanciate the
remote API with "sim=remApi('remoteApi','extApi.h');" instead of "sim=remApi('remoteApi');"

Finally, if you wish to rebuild the prototype file, you will have to comply with above conditions, then type:

loadlibrary('remoteApi','extApi.h','mfilename','remoteApiProto')

For more examples, have a look at the python folder: language is different but principles are the same