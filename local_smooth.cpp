#include "common.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>

namespace PMP = CGAL::Polygon_mesh_processing;

void local_smooth(const double* vertices,
	int vertCount,
	const int* faces,
	int faceCount,
	const int* faceArea,
	int faceAreaCount,
	double* result,
	int* resultFaces,
	int iterations,
	int use_safety_constraints,
	int use_Delaunay_flips,
	int do_project) {
	typedef CGAL::Epick::Point_3 Point;
	typedef CGAL::SM_Vertex_index Vid;

	const auto V = convert_matrix(vertices, vertCount, 3);
	const auto F = convert_matrix(faces, faceCount, 3);

	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef CGAL::Surface_mesh<K::Point_3> Mesh;
	typedef boost::graph_traits<Mesh>::face_descriptor     face_descriptor;

	auto mesh = Mesh();

	std::cout << "VA rows count: " << V.rows() << std::endl;
	std::cout << "VA cols count: " << F.cols() << std::endl;

	std::map<int, Vid> map = std::map<int, Vid>();

	for (size_t i = 0; i < V.rows(); i++)
	{
		auto point = Point(V(i, 0), V(i, 1), V(i, 2));
		map[i] = mesh.add_vertex(point);
	}

	std::vector<face_descriptor> patch;
	auto facesAreaSet = std::unordered_set<int>();
	for (size_t i = 0; i < faceAreaCount; i++)
	{
		auto faceId = faceArea[i];
		facesAreaSet.insert(faceId);
	}
	for (size_t i = 0; i < F.rows(); i++)
	{
		face_descriptor face = mesh.add_face(map[F(i, 0)], map[F(i, 1)], map[F(i, 2)]);
		std::unordered_set<int>::const_iterator got = facesAreaSet.find(i);
		if (got != facesAreaSet.end())
		{
			patch.push_back(face);
		}
	}

	//auto blah = FaceRange
	if (faceAreaCount == 0)
	{
		PMP::smooth_mesh(mesh, PMP::parameters::number_of_iterations(iterations)
			.use_safety_constraints((bool)use_safety_constraints).use_Delaunay_flips((bool)use_Delaunay_flips).do_project((bool)do_project)); // authorize all moves
	}
	else
	{
		PMP::smooth_mesh(patch, mesh, PMP::parameters::number_of_iterations(iterations)
			.use_safety_constraints((bool)use_safety_constraints).use_Delaunay_flips((bool)use_Delaunay_flips).do_project((bool)do_project)); // authorize all moves
	}
	

	for (size_t i = 0; i < V.rows(); i++)
	{
		const auto &point = mesh.point(map[i]);
		result[i * 3] = point.x();
		result[i * 3 + 1] = point.y();
		result[i * 3 + 2] = point.z();
	}
	
	int faceCounter = 0;
	int facesArraySize = faceCount * 3;
	for (Mesh::Face_index face_index : mesh.faces()) {
		CGAL::Vertex_around_face_circulator<Mesh> vcirc(mesh.halfedge(face_index), mesh), done(vcirc);
		do
		{
			if (faceCounter >= facesArraySize)
			{
				throw "FaceCounter overflow";
			}
			uint32_t vid = *vcirc++;
			resultFaces[faceCounter] = (int)vid;
			faceCounter++;
		} while (vcirc != done);
	}

	for (size_t i = 0; i < V.rows(); i++)
	{
		const auto& point = mesh.point(map[i]);
		result[i * 3] = point.x();
		result[i * 3 + 1] = point.y();
		result[i * 3 + 2] = point.z();
	}
}