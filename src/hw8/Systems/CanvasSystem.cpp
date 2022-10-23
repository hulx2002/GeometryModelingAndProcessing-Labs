#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>

using namespace Ubpa;

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef CGAL::Delaunay_triangulation_2<K>  Triangulation;
typedef Triangulation::Edge_iterator  Edge_iterator;
typedef Triangulation::Vertex_iterator  Vertex_iterator;
typedef Triangulation::Point          Point;

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");

			static pointf2 region_p0 = pointf2(0.f, 0.f);
			static pointf2 region_p1 = pointf2(100.f, 100.f);
			static int points_number = 0;
			ImGui::InputInt("Enter number of random points", &points_number);
			if (ImGui::Button("Insert random points")) {
				data->points.clear();
				for (int i = 0; i < points_number; i++) {
					float x = region_p0[0] + 1.f * rand() / RAND_MAX * (region_p1[0] - region_p0[0]);
					float y = region_p0[1] + 1.f * rand() / RAND_MAX * (region_p1[1] - region_p0[1]);
					data->points.push_back(pointf2(x, y));
				}
			}

			static bool opt_enable_voronoi_diagram = false;
			ImGui::Checkbox("Show Voronoi Diagram", &opt_enable_voronoi_diagram);
			Triangulation T;
			std::vector<Point> delaunay_triangulation_points;
			for (int i = 0; i < data->points.size(); i++)
				delaunay_triangulation_points.push_back(Point(data->points[i][0], data->points[i][1]));
			T.insert(delaunay_triangulation_points.begin(), delaunay_triangulation_points.end());

			static int iterations = 0;
			ImGui::InputInt("Enter iterations", &iterations);
			if (ImGui::Button("Start Lloyd algorithm")) {

			}

			// Typically you would use a BeginChild()/EndChild() pair to benefit from a clipping region + own scrolling.
			// Here we demonstrate that this can be replaced by simple offsetting + custom drawing + PushClipRect/PopClipRect() calls.
			// To use a child window instead we could use, e.g:
			//      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));      // Disable padding
			//      ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(50, 50, 50, 255));  // Set a background color
			//      ImGui::BeginChild("canvas", ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoMove);
			//      ImGui::PopStyleColor();
			//      ImGui::PopStyleVar();
			//      [...]
			//      ImGui::EndChild();

			// Using InvisibleButton() as a convenience 1) it will advance the layout cursor and 2) allows us to use IsItemHovered()/IsItemActive()
			ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();      // ImDrawList API uses screen coordinates!
			ImVec2 canvas_sz = ImGui::GetContentRegionAvail();   // Resize canvas to what's available
			if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
			if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
			ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

			// Draw border and background color
			ImGuiIO& io = ImGui::GetIO();
			ImDrawList* draw_list = ImGui::GetWindowDrawList();
			draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
			draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

			// This will catch our interactions
			ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
			const bool is_hovered = ImGui::IsItemHovered(); // Hovered
			const bool is_active = ImGui::IsItemActive();   // Held
			const ImVec2 origin(canvas_p0.x + data->scrolling[0], canvas_p0.y + data->scrolling[1]); // Lock scrolled origin
			const pointf2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);

			// Add first and second point
			if (is_hovered && !data->adding_line && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			{
				region_p0 = mouse_pos_in_canvas;
				region_p1 = mouse_pos_in_canvas;
				data->adding_line = true;
			}
			if (data->adding_line)
			{
				region_p1 = mouse_pos_in_canvas;
				if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
					data->adding_line = false;
			}

			// Pan (we use a zero mouse threshold when there's no context menu)
			// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
			const float mouse_threshold_for_pan = data->opt_enable_context_menu ? -1.0f : 0.0f;
			if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan))
			{
				data->scrolling[0] += io.MouseDelta.x;
				data->scrolling[1] += io.MouseDelta.y;
			}

			// Context menu (under default mouse threshold)
			ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
			if (data->opt_enable_context_menu && ImGui::IsMouseReleased(ImGuiMouseButton_Right) && drag_delta.x == 0.0f && drag_delta.y == 0.0f)
				ImGui::OpenPopupContextItem("context");
			if (ImGui::BeginPopup("context"))
			{
				if (data->adding_line)
					data->points.resize(data->points.size() - 1);
				data->adding_line = false;
				if (ImGui::MenuItem("Remove one", NULL, false, data->points.size() > 0)) { data->points.resize(data->points.size() - 1); }
				if (ImGui::MenuItem("Remove all", NULL, false, data->points.size() > 0)) { data->points.clear(); }
				ImGui::EndPopup();
			}

			// Draw grid + all lines in the canvas
			draw_list->PushClipRect(canvas_p0, canvas_p1, true);
			if (data->opt_enable_grid)
			{
				const float GRID_STEP = 64.0f;
				for (float x = fmodf(data->scrolling[0], GRID_STEP); x < canvas_sz.x; x += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
				for (float y = fmodf(data->scrolling[1], GRID_STEP); y < canvas_sz.y; y += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(200, 200, 200, 40));
			}
			/*
			for (int n = 0; n < data->points.size(); n += 2)
				draw_list->AddLine(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), ImVec2(origin.x + data->points[n + 1][0], origin.y + data->points[n + 1][1]), IM_COL32(255, 255, 0, 255), 2.0f);
			*/
			draw_list->AddRect(ImVec2(origin.x + region_p0[0], origin.y + region_p0[1]), ImVec2(origin.x + region_p1[0], origin.y + region_p1[1]), IM_COL32(255, 255, 0, 255));
			for (int n = 0; n < data->points.size(); n++)
				draw_list->AddCircle(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), 3, IM_COL32(255, 0, 255, 255));
			if (opt_enable_voronoi_diagram) {
				Edge_iterator eit = T.edges_begin();
				for (; eit != T.edges_end(); ++eit) {
					CGAL::Object o = T.dual(eit);
					if (CGAL::object_cast<K::Segment_2>(&o)) {
						auto segment_temp = CGAL::object_cast<K::Segment_2>(&o);
						draw_list->AddLine(ImVec2(origin.x + segment_temp->source().x(), origin.y + segment_temp->source().y()), ImVec2(origin.x + segment_temp->target().x(), origin.y + segment_temp->target().y()), IM_COL32(0, 255, 255, 255));
					}
					else if (CGAL::object_cast<K::Ray_2>(&o)) {
						auto ray_temp = CGAL::object_cast<K::Ray_2>(&o);
						draw_list->AddLine(ImVec2(origin.x + ray_temp->source().x(), origin.y + ray_temp->source().y()), ImVec2(origin.x + ray_temp->source().x() + ray_temp->direction().dx(), origin.y + ray_temp->source().y() + ray_temp->direction().dy()), IM_COL32(0, 255, 255, 255));
					}
				}
			}
			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}