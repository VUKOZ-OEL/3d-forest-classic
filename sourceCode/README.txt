3DFOREST application
created as a part of PhD thesis at VUKOZ Czech republic.
Published under terms ofGnu/GPL v3 licence.
created by: Jan trochta j.trochta@gmail.com

INSTALLATION
	
	For successful compilation from source are needed those libraries:
	Qt 5.x, Eigen, Boost, Flann, Qhull, libusb 1.0, libLAS, VTK 9.x, PCL >= 1.11.
	
	For Linux OSs, you can download Qt 5.x, Eigen, Boost, Flann, Qhull, libusb 1.0 from official Linux repositories (e.g. using apt, dnf, etc.). Then you have to compile libLAS, vtk and pcl source code with Qt support, or you can use these library already compiled which are attached in compressed folder of 3D forest compiled.
	
	For Windows OSs, you can install Qt with the qt online installer (https://www.qt.io/download-qt-installer?hsCtaTracking=99d9dd4f-5681-48d2-b096-470725510d34%7C074ddad0-fdef-4e53-8aa8-5e8a876d6ab4).
	To compile Eigen, Boost, Flann, Qhull, libusb 1.0 and libLAS you can use vcpkg repository and msvc v142 (Visual Studio 2019), for vtk and pcl you have to compile the source codes with Qt support.
	3D Forest was also compiled with msvc v142.	
	Or you can install compiled application where you want with Windows installer file (.msi).
