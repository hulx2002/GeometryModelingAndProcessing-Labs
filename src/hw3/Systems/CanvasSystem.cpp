#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>

#include <Eigen/Dense>

#include <math.h>

using namespace Ubpa;
using namespace std;
using namespace Eigen;

#define PI acos(-1)

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");

			// 选择参数化方法
			static int e = 0;
			ImGui::RadioButton("Stop fitting", &e, 0);
			ImGui::RadioButton("Uniform parameterization", &e, 1);
			ImGui::RadioButton("Chordal parameterization", &e, 2);
			ImGui::RadioButton("Centripetal parameterization", &e, 3);
			ImGui::RadioButton("Foley parameterization", &e, 4);
			// 设置参数值
			static float sigma = 1.0f;
			ImGui::SliderFloat("sigma", &sigma, 1.0f, 20.0f, "%.3f");

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

			// 参数化
			vector<float> t;
			t.clear();
			if (e > 0 && data->points.size() > 0)
			{
				// Uniform参数化
				if (e == 1)
				{
					int n = data->points.size();
					for (int i = 0; i < n; i++)
						t.push_back(1.0f * i);
				}
				// Chordal参数化
				else if (e == 2)
				{
					float temp = 100.0f;
					int n = data->points.size();
					t.push_back(0.0f);
					for (int i = 1; i < n; i++)
						t.push_back(t.back() + sqrt(pow(data->points[i][0] - data->points[i - 1][0], 2) + pow(data->points[i][1] - data->points[i - 1][1], 2)) / temp);
				}
				// Centripetal参数化
				else if (e == 3)
				{
					float temp = 10.0f;
					int n = data->points.size();
					t.push_back(0.0f);
					for (int i = 1; i < n; i++)
						t.push_back(t.back() + sqrt(sqrt(pow(data->points[i][0] - data->points[i - 1][0], 2) + pow(data->points[i][1] - data->points[i - 1][1], 2))) / temp);
				}
				// Foley参数化
				else if (e == 4)
				{
					float temp = 100.0f;
					int n = data->points.size();
					// 计算距离
					vector<float> dist;
					dist.clear();
					for (int i = 0; i < n - 1; i++)
						dist.push_back(sqrt(pow(data->points[i + 1][0] - data->points[i][0], 2) + pow(data->points[i + 1][1] - data->points[i][1], 2)));
					// 计算夹角
					vector<float> alpha;
					alpha.clear();
					alpha.push_back(0.0f);
					for (int i = 1; i < n - 1; i++)
					{
						alpha.push_back(acos(((data->points[i - 1][0] - data->points[i][0]) * (data->points[i + 1][0] - data->points[i][0]) + (data->points[i - 1][1] - data->points[i][1]) * (data->points[i + 1][1] - data->points[i][1])) / dist[i] / dist[i - 1]));
						alpha.back() = (PI - alpha.back() < PI / 2.0f) ? (PI - alpha.back()) : (PI / 2.0f);
					}
					// 计算参数
					t.push_back(0.0f);
					for (int i = 0; i < n - 1; i++)
					{
						if (i == 0 && i == n - 2)
							t.push_back(t.back() + dist[i] / temp);
						else if (i == 0)
							t.push_back(t.back() + dist[i] * (1.0f + 3.0f / 2.0f * alpha[i + 1] * dist[i] / (dist[i] + dist[i + 1])) / temp);
						else if (i == n - 2)
							t.push_back(t.back() + dist[i] * (1.0f + 3.0f / 2.0f * alpha[i] * dist[i - 1] / (dist[i - 1] + dist[i])) / temp);
						else
							t.push_back(t.back() + dist[i] * (1.0f + 3.0f / 2.0f * alpha[i] * dist[i - 1] / (dist[i - 1] + dist[i]) + 3.0f / 2.0f * alpha[i + 1] * dist[i] / (dist[i] + dist[i + 1])) / temp);
					}
				}
			}

			// 拟合
			// Gauss基函数插值
			vector<pointf2> fitting_points;
			fitting_points.clear();
			if (e > 0 && data->points.size() > 0)
			{
				int n = data->points.size();
				// 分别计算x和y关于t的系数
				MatrixXf g(n, n);
				for (size_t row = 0; row < n; row++)
					for (size_t col = 0; col < n; col++)
						g(row, col) = exp(-pow(t[row] - t[col], 2) / (2 * pow(sigma, 2)));
				MatrixXf f_x(n, 1);
				for (size_t row = 0; row < n; row++)
					f_x(row, 0) = data->points[row][0];
				MatrixXf b_x = g.colPivHouseholderQr().solve(f_x);
				MatrixXf f_y(n, 1);
				for (size_t row = 0; row < n; row++)
					f_y(row, 0) = data->points[row][1];
				MatrixXf b_y = g.colPivHouseholderQr().solve(f_y);
				// 对参数t均匀采样
				for (float t_iter = t[0]; t_iter <= t[n - 1]; t_iter += 0.01f)
				{
					float x = 0.0f;
					for (size_t i = 0; i < n; i++)
						x += b_x(i, 0) * exp(-pow(t_iter - t[i], 2) / (2 * pow(sigma, 2)));
					float y = 0.0f;
					for (size_t i = 0; i < n; i++)
						y += b_y(i, 0) * exp(-pow(t_iter - t[i], 2) / (2 * pow(sigma, 2)));
					fitting_points.push_back(pointf2(x, y));
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
