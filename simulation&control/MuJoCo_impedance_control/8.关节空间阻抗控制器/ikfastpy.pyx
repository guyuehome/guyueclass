#This is a Cython file and extracts the relevant classes from the C++ header file.

# distutils: language = c++
# distutils: sources = ikfast_wrapper.cpp

cdef extern from "<vector>" namespace "std":
    cdef cppclass vector[T]:
        cppclass iterator:
            T operator*()
            iterator operator++()
            bint operator==(iterator)
            bint operator!=(iterator)
        vector()
        void push_back(T&)
        T& operator[](int)
        T& at(int)
        iterator begin()
        iterator end()

cdef extern from "Kinematics.hpp" namespace "robots":
    cdef cppclass Kinematics:
        Kinematics()
        int num_of_joints, num_free_parameters
        vector[float] forward(vector[float] joint_config);
        vector[float] inverse(vector[float] ee_pose);

cdef class PyKinematics:
    cdef Kinematics *thisptr      # hold a C++ instance which we're wrapping
    def __cinit__(self):
        self.thisptr = new Kinematics()
    def __dealloc__(self):
        del self.thisptr
    def getDOF(self):
        return self.thisptr.num_of_joints
    def forward(self,vector[float] joint_config):
        return self.thisptr.forward(joint_config)
    def inverse(self,vector[float] ee_pose):
        return self.thisptr.inverse(ee_pose)