#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>

#include <Eigen/Dense>

#include <math.h>

using namespace Ubpa;
using namespace std;
using namespace Eigen;

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");

			static int e = 0;
			ImGui::RadioButton("Stop fitting", &e, 0);
			ImGui::RadioButton("Start fitting", &e, 1);

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
				data->points.push_back(mouse_pos_in_canvas);
				data->adding_line = true;
			}
			if (data->adding_line)
			{
				if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
					data->adding_line = false;
			}

			vector<float> t;
			t.clear();
			if (e > 0 && data->points.size() > 0)
			{
				float temp = 100.0f;
				int n = data->points.size() - 1;
				t.push_back(0.0f);
				for (int i = 1; i <= n; i++)
					t.push_back(t.back() + sqrt(pow(data->points[i][0] - data->points[i - 1][0], 2) + pow(data->points[i][1] - data->points[i - 1][1], 2)) / temp);
			}

			vector<pointf2> fitting_points;
			fitting_points.clear();
			if (e > 0 && data->points.size() == 2)
			{
				for (float t_iter = t[0]; t_iter <= t[1]; t_iter += 0.01f)
				{
					float x = data->points[1][0] / (t[1] - t[0]) * (t_iter - t[0]) + data->points[0][0] / (t[1] - t[0]) * (t[1] - t_iter);
					float y = data->points[1][1] / (t[1] - t[0]) * (t_iter - t[0]) + data->points[0][1] / (t[1] - t[0]) * (t[1] - t_iter);
					fitting_points.push_back(pointf2(x, y));
				}
			}
			else if (e > 0 && data->points.size() > 2)
			{
				int n = data->points.size() - 1;
				vector<float> h;
				h.clear();
				for (int i = 0; i <= n - 1; i++)
					h.push_back(t[i + 1] - t[i]);
				vector<float> u;
				u.clear();
				u.push_back(0.0f);
				for (int i = 1; i <= n - 1; i++)
					u.push_back(2 * (h[i] + h[i - 1]));
				vector<float> b_x;
				b_x.clear();
				for (int i = 0; i <= n - 1; i++)
					b_x.push_back(6 / h[i] * (data->points[i + 1][0] - data->points[i][0]));
				vector<float> b_y;
				b_y.clear();
				for (int i = 0; i <= n - 1; i++)
					b_y.push_back(6 / h[i] * (data->points[i + 1][1] - data->points[i][1]));
				vector<float> v_x;
				v_x.clear();
				v_x.push_back(0.0f);
				for (int i = 1; i <= n - 1; i++)
					v_x.push_back(b_x[i] - b_x[i - 1]);
				vector<float> v_y;
				v_y.clear();
				v_y.push_back(0.0f);
				for (int i = 1; i <= n - 1; i++)
					v_y.push_back(b_y[i] - b_y[i - 1]);
				MatrixXf A = MatrixXf::Zero(n - 1, n - 1);
				for (size_t i = 0; i < n - 1; i++)
					A(i, i) = u[i + 1];
				for (size_t i = 0; i < n - 2; i++)
				{
					A(i, i + 1) = h[i + 1];
					A(i + 1, i) = h[i + 1];
				}
				MatrixXf V_x(n - 1, 1);
				for (size_t i = 0; i < n - 1; i++)
					V_x(i, 0) = v_x[i + 1];
				MatrixXf V_y(n - 1, 1);
				for (size_t i = 0; i < n - 1; i++)
					V_y(i, 0) = v_y[i + 1];
				MatrixXf M_x = A.colPivHouseholderQr().solve(V_x);
				MatrixXf M_y = A.colPivHouseholderQr().solve(V_y);
				for (int i = 0; i < n; i++)
				{
					for (float t_iter = t[i]; t_iter <= t[i + 1]; t_iter += 0.01f)
					{
						float x, y;
						if (i == 0)
						{
							x = M_x(i, 0) / (6 * h[i]) * pow(t_iter - t[i], 3) + (data->points[i + 1][0] / h[i] - M_x(i, 0) * h[i] / 6) * (t_iter - t[i]) + data->points[i][0] / h[i] * (t[i + 1] - t_iter);
							y = M_y(i, 0) / (6 * h[i]) * pow(t_iter - t[i], 3) + (data->points[i + 1][1] / h[i] - M_y(i, 0) * h[i] / 6) * (t_iter - t[i]) + data->points[i][1] / h[i] * (t[i + 1] - t_iter);
						}
						else if (i == n - 1)
						{
							x = M_x(i - 1, 0) / (6 * h[i]) * pow(t[i + 1] - t_iter, 3) + data->points[i + 1][0] / h[i] * (t_iter - t[i]) + (data->points[i][0] / h[i] - M_x(i - 1, 0) * h[i] / 6) * (t[i + 1] - t_iter);
							y = M_y(i - 1, 0) / (6 * h[i]) * pow(t[i + 1] - t_iter, 3) + data->points[i + 1][1] / h[i] * (t_iter - t[i]) + (data->points[i][1] / h[i] - M_y(i - 1, 0) * h[i] / 6) * (t[i + 1] - t_iter);
						}
						else {
							x = M_x(i - 1, 0) / (6 * h[i]) * pow(t[i + 1] - t_iter, 3) + M_x(i, 0) / (6 * h[i]) * pow(t_iter - t[i], 3) + (data->points[i + 1][0] / h[i] - M_x(i, 0) * h[i] / 6) * (t_iter - t[i]) + (data->points[i][0] / h[i] - M_x(i - 1, 0) * h[i] / 6) * (t[i + 1] - t_iter);
							y = M_y(i - 1, 0) / (6 * h[i]) * pow(t[i + 1] - t_iter, 3) + M_y(i, 0) / (6 * h[i]) * pow(t_iter - t[i], 3) + (data->points[i + 1][1] / h[i] - M_y(i, 0) * h[i] / 6) * (t_iter - t[i]) + (data->points[i][1] / h[i] - M_y(i - 1, 0) * h[i] / 6) * (t[i + 1] - t_iter);
						}
						fitting_points.push_back(pointf2(x, y));
					}
				}
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
					data->points.resize(data->points.size() - 2);
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
			// 标记原点
			draw_list->AddCircle(ImVec2(origin.x, origin.y), 3, IM_COL32(255, 0, 0, 255), 0, 2.0f);
			// 标记输入点集
			for (int n = 0; n < data->points.size(); n++)
				draw_list->AddCircle(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), 3, IM_COL32(255, 255, 0, 255), 0, 2.0f);
			// 连接采样点，绘制拟合函数图像
			for (int n = 0; n + 1 < fitting_points.size(); n++)
				draw_list->AddLine(ImVec2(origin.x + fitting_points[n][0], origin.y + fitting_points[n][1]), ImVec2(origin.x + fitting_points[n + 1][0], origin.y + fitting_points[n + 1][1]), IM_COL32(255, 255, 0, 255), 2.0f);
			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}
