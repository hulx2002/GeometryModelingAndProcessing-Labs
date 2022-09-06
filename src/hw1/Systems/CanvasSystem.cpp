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

			// 选择拟合方法
			static int e = 0;
			ImGui::RadioButton("stop fitting", &e, 0);
			ImGui::RadioButton("interpolation polynomial", &e, 1);
			ImGui::RadioButton("interpolation gauss", &e, 2);
			ImGui::RadioButton("approach leastsquare", &e, 3);
			ImGui::RadioButton("approach ridgeregression", &e, 4);
			// 设置参数值
			static float sigma = 1.0f;
			ImGui::SliderFloat("sigma", &sigma, 1.0f, 20.0f, "%.3f");
			static int m = 0;
			ImGui::SliderInt("m", &m, 0, 10);
			static float lambda = 0.0f;
			ImGui::SliderFloat("lambda", &lambda, 0.0f, 1.0f, "%.3f");

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

			vector<pointf2> fitting_points;
			fitting_points.clear();
			if (e > 0 && data->points.size() > 0)
			{
				// 插值型拟合方法，使用多项式函数
				if (e == 1)
				{
					float temp = 100.0f;
					int n = data->points.size();
					// 求解线性方程组，得到拟合函数的系数
					MatrixXf B(n, n);
					for (size_t row = 0; row < n; row++)
						for (size_t col = 0; col < n; col++)
							B(row, col) = pow(data->points[row][0] / temp, col);
					MatrixXf f(n, 1);
					for (size_t row = 0; row < n; row++)
						f(row, 0) = data->points[row][1] / temp;
					MatrixXf alpha = B.colPivHouseholderQr().solve(f);
					// 沿着x轴均匀采样
					for (float x = canvas_p0.x - origin.x; x < canvas_p1.x - origin.x; x += 1.0f)
					{
						float y = 0.0f;
						for (size_t i = 0; i < n; i++)
							y += alpha(i, 0) * pow(x / temp, i);
						y = y * temp;
						fitting_points.push_back(pointf2(x, y));
					}
				}
				// 插值型拟合方法，使用Gauss基函数
				else if (e == 2)
				{
					int n = data->points.size();
					MatrixXf g(n, n);
					// 求解线性方程组，得到拟合函数的系数
					for (size_t row = 0; row < n; row++)
						for (size_t col = 0; col < n; col++)
							g(row, col) = exp(-pow(data->points[row][0] - data->points[col][0], 2) / (2 * pow(sigma, 2)));
					MatrixXf f(n, 1);
					for (size_t row = 0; row < n; row++)
						f(row, 0) = data->points[row][1];
					MatrixXf b = g.colPivHouseholderQr().solve(f);
					// 沿着x轴均匀采样
					for (float x = canvas_p0.x - origin.x; x < canvas_p1.x - origin.x; x += 1.0f)
					{
						float y = 0.0f;
						for (size_t i = 0; i < n; i++)
							y += b(i, 0) * exp(-pow(x - data->points[i][0], 2) / (2 * pow(sigma, 2)));
						fitting_points.push_back(pointf2(x, y));
					}
				}
				// 逼近型拟合方法，使用最小二乘法
				else if (e == 3)
				{
					float temp = 100.0f;
					int n = data->points.size();
					m = m < n ? m : n - 1;
					// 求解线性方程组，得到拟合函数的系数
					MatrixXf A(m + 1, m + 1);
					for (size_t row = 0; row < m + 1; row++)
						for (size_t col = 0; col < m + 1; col++)
						{
							A(row, col) = 0.0f;
							for (int i = 0; i < n; i++)
								A(row, col) += pow(data->points[i][0] / temp, row + col);
						}
					MatrixXf b(m + 1, 1);
					for (size_t row = 0; row < m + 1; row++)
					{
						b(row, 0) = 0.0f;
						for (int i = 0; i < n; i++)
							b(row, 0) += pow(data->points[i][0] / temp, row) * data->points[i][1] / temp;
					}
					MatrixXf alpha = A.colPivHouseholderQr().solve(b);
					// 沿着x轴均匀采样
					for (float x = canvas_p0.x - origin.x; x < canvas_p1.x - origin.x; x += 1.0f)
					{
						float y = 0.0f;
						for (size_t i = 0; i < m + 1; i++)
							y += alpha(i, 0) * pow(x / temp, i);
						y = y * temp;
						fitting_points.push_back(pointf2(x, y));
					}
				}
				// 逼近型拟合方法，使用岭回归
				else if (e == 4)
				{
					float temp = 100.0f;
					int n = data->points.size();
					m = m < n ? m : n - 1;
					// 求解线性方程组，得到拟合函数的系数
					MatrixXf A(m + 1, m + 1);
					for (size_t row = 0; row < m + 1; row++)
					{
						for (size_t col = 0; col < m + 1; col++)
						{
							A(row, col) = 0.0f;
							for (int i = 0; i < n; i++)
								A(row, col) += pow(data->points[i][0] / temp, row + col);
						}
						A(row, row) += lambda;
					}
					MatrixXf b(m + 1, 1);
					for (size_t row = 0; row < m + 1; row++)
					{
						b(row, 0) = 0.0f;
						for (int i = 0; i < n; i++)
							b(row, 0) += pow(data->points[i][0] / temp, row) * data->points[i][1] / temp;
					}
					MatrixXf alpha = A.colPivHouseholderQr().solve(b);
					// 沿着x轴均匀采样
					for (float x = canvas_p0.x - origin.x; x < canvas_p1.x - origin.x; x += 1.0f)
					{
						float y = 0.0f;
						for (size_t i = 0; i < m + 1; i++)
							y += alpha(i, 0) * pow(x / temp, i);
						y = y * temp;
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
