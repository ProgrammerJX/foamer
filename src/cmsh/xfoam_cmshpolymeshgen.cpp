#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"

#include <boost/filesystem.hpp>

#include <fstream>
#include <iostream>

namespace fs = boost::filesystem;

namespace
{
void writeFoamHeader(std::ofstream& f,
                     const std::string& className,
                     const std::string& location,
                     const std::string& object)
{
	f << "FoamFile\n{\n"
	  << "    version     2.0;\n"
	  << "    format      ascii;\n"
	  << "    class       " << className << ";\n"
	  << "    location    \"" << location << "\";\n"
	  << "    object      " << object << ";\n"
	  << "}\n"
	  << "// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //\n\n";
}
} // namespace

bool XFoam_CMshPolyMeshGen::writeToDir(const XFoam_String& dir) const
{
	const std::string dirStr(static_cast<const std::string&>(dir));
	fs::path path(dirStr);
	boost::system::error_code ec;
	fs::create_directories(path, ec);
	if (ec)
	{
		std::cerr << "cmsh polyMesh writeToDir: failed to create " << dirStr
		          << ": " << ec.message() << "\n";
		return false;
	}

	auto childPath = [&](const char* name) -> std::string {
		return (path / name).string();
	};

	// points
	{
		std::ofstream f(childPath("points"));
		if (!f) return false;
		writeFoamHeader(f, "vectorField", "constant/polyMesh", "points");
		f << points.size() << "\n(\n";
		for (const auto& p : points)
		{
			f << "(" << p.x() << " " << p.y() << " " << p.z() << ")\n";
		}
		f << ")\n";
	}

	// faces
	{
		std::ofstream f(childPath("faces"));
		if (!f) return false;
		writeFoamHeader(f, "faceList", "constant/polyMesh", "faces");
		f << faces.size() << "\n(\n";
		for (const auto& face : faces)
		{
			f << face.verts.size() << "(";
			for (std::size_t i = 0; i < face.verts.size(); ++i)
			{
				if (i) f << " ";
				f << face.verts[i];
			}
			f << ")\n";
		}
		f << ")\n";
	}

	// owner
	{
		std::ofstream f(childPath("owner"));
		if (!f) return false;
		writeFoamHeader(f, "labelList", "constant/polyMesh", "owner");
		f << owner.size() << "\n(\n";
		for (int v : owner) f << v << "\n";
		f << ")\n";
	}

	// neighbour
	{
		std::ofstream f(childPath("neighbour"));
		if (!f) return false;
		writeFoamHeader(f, "labelList", "constant/polyMesh", "neighbour");
		f << neighbour.size() << "\n(\n";
		for (int v : neighbour) f << v << "\n";
		f << ")\n";
	}

	// boundary
	{
		std::ofstream f(childPath("boundary"));
		if (!f) return false;
		writeFoamHeader(f, "polyBoundaryMesh", "constant/polyMesh", "boundary");
		f << patches.size() << "\n(\n";
		for (const auto& p : patches)
		{
			f << "    " << p.name << "\n"
			  << "    {\n"
			  << "        type        " << p.type << ";\n"
			  << "        nFaces      " << p.nFaces << ";\n"
			  << "        startFace   " << p.startFace << ";\n"
			  << "    }\n";
		}
		f << ")\n";
	}

	return true;
}
