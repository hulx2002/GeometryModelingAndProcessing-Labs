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

			// 选择输入型值点、修改拖动型值点或编辑型值点处的切线信息
			static int e_1 = 0;
			ImGui::RadioButton("Add points", &e_1, 0);
			ImGui::RadioButton("Edit points", &e_1, 1);
			ImGui::RadioButton("Edit tangents", &e_1, 2);
			// 编辑型值点处切线信息时，选择曲线几何连续性阶数
			static int e_2 = 1;
			ImGui::RadioButton("G1", &e_2, 1);
			ImGui::RadioButton("G0", &e_2, 0);

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

			static int data_id = -1;
			static int control_id = -1;
			static vector<pointf2> control_points;
			// 输入型值点
			if (e_1 == 0)
			{
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
			}
			// 修改拖动型值点
			else if (e_1 == 1)
			{
				int n = data->points.size() - 1;
				// 鼠标左键点击
				if (is_hovered && data_id == -1 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
				{
					// 寻找距离点击位置最近的型值点
					float min_dist = 3.0f;
					for (int i = 0; i <= n; i++)
						if (sqrt(pow(mouse_pos_in_canvas[0] - data->points[i][0], 2) + pow(mouse_pos_in_canvas[1] - data->points[i][1], 2)) < min_dist)
						{
							min_dist = sqrt(pow(mouse_pos_in_canvas[0] - data->points[i][0], 2) + pow(mouse_pos_in_canvas[1] - data->points[i][1], 2));
							data_id = i;
						}
				}
				// 鼠标拖动
				if (data_id > -1)
				{
					// 根据拖动位置修改型值点位置
					data->points[data_id] = mouse_pos_in_canvas;
					// 鼠标左键松开
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// 修改结束
						data_id = -1;
				}
			}
			// 编辑型值点处切线信息，曲线1阶几何连续
			else if (e_1 == 2 && e_2 == 1)
			{
				int n = data->points.size() - 1;
				// 鼠标左键点击
				if (is_hovered && data_id == -1 && control_id == -1 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
				{
					// 寻找距离点击位置最近的型值点或切线控制点
					float min_dist = 3.0f;
					for (int i = 0; i <= n; i++)
						if (sqrt(pow(mouse_pos_in_canvas[0] - data->points[i][0], 2) + pow(mouse_pos_in_canvas[1] - data->points[i][1], 2)) < min_dist)
						{
							min_dist = sqrt(pow(mouse_pos_in_canvas[0] - data->points[i][0], 2) + pow(mouse_pos_in_canvas[1] - data->points[i][1], 2));
							data_id = i;
						}
					for (int i = 0; i <= 2 * n - 1; i++)
						if (sqrt(pow(mouse_pos_in_canvas[0] - control_points[i][0], 2) + pow(mouse_pos_in_canvas[1] - control_points[i][1], 2)) < min_dist)
						{
							min_dist = sqrt(pow(mouse_pos_in_canvas[0] - control_points[i][0], 2) + pow(mouse_pos_in_canvas[1] - control_points[i][1], 2));
							control_id = i;
							data_id = -1;
						}
				}
				// 鼠标拖动，选中型值点
				if (data_id > -1)
				{
					float delta_x = mouse_pos_in_canvas[0] - data->points[data_id][0];
					float delta_y = mouse_pos_in_canvas[1] - data->points[data_id][1];
					// 根据拖动位置修改型值点位置
					data->points[data_id] = mouse_pos_in_canvas;
					// 更新两侧切线控制点位置
					if (data_id == 0)
					{
						control_points[0][0] += delta_x;
						control_points[0][1] += delta_y;
					}
					else if (data_id == n)
					{
						control_points[2 * n - 1][0] += delta_x;
						control_points[2 * n - 1][1] += delta_y;
					}
					else
					{
						control_points[2 * data_id - 1][0] += delta_x;
						control_points[2 * data_id - 1][1] += delta_y;
						control_points[2 * data_id][0] += delta_x;
						control_points[2 * data_id][1] += delta_y;
					}
					// 鼠标左键松开
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// 修改结束
						data_id = -1;
				}
				// 鼠标拖动，选中切线控制点
				if (control_id > -1)
				{
					// 根据拖动位置修改切线控制点信息
					control_points[control_id] = mouse_pos_in_canvas;
					// 更新另一侧切线控制点信息，保持另一侧切线大小不变且两侧切线方向连续
					if (control_id != 0 && control_id != 2 * n - 1)
					{
						if (control_id % 2 == 0)
						{
							float dist_1 = sqrt(pow(control_points[control_id][0] - data->points[control_id / 2][0], 2) + pow(control_points[control_id][1] - data->points[control_id / 2][1], 2));
							float dist_2 = sqrt(pow(control_points[control_id - 1][0] - data->points[control_id / 2][0], 2) + pow(control_points[control_id - 1][1] - data->points[control_id / 2][1], 2));
							control_points[control_id - 1][0] = ((dist_1 + dist_2) * data->points[control_id / 2][0] - dist_2 * control_points[control_id][0]) / dist_1;
							control_points[control_id - 1][1] = ((dist_1 + dist_2) * data->points[control_id / 2][1] - dist_2 * control_points[control_id][1]) / dist_1;
						}
						else
						{
							float dist_1 = sqrt(pow(control_points[control_id][0] - data->points[(control_id + 1) / 2][0], 2) + pow(control_points[control_id][1] - data->points[(control_id + 1) / 2][1], 2));
							float dist_2 = sqrt(pow(control_points[control_id + 1][0] - data->points[(control_id + 1) / 2][0], 2) + pow(control_points[control_id + 1][1] - data->points[(control_id + 1) / 2][1], 2));
							control_points[control_id + 1][0] = ((dist_1 + dist_2) * data->points[(control_id + 1) / 2][0] - dist_2 * control_points[control_id][0]) / dist_1;
							control_points[control_id + 1][1] = ((dist_1 + dist_2) * data->points[(control_id + 1) / 2][1] - dist_2 * control_points[control_id][1]) / dist_1;
						}
					}
					// 鼠标左键松开
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// 修改结束
						control_id = -1;
				}
			}
			// 编辑型值点处切线信息，曲线0阶几何连续
			else if (e_1 == 2 && e_2 == 0)
			{
				int n = data->points.size() - 1;
				// 鼠标左键点击
				if (is_hovered && data_id == -1 && control_id == -1 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
				{
					// 寻找距离点击位置最近的型值点或切线控制点
					float min_dist = 3.0f;
					for (int i = 0; i <= n; i++)
						if (sqrt(pow(mouse_pos_in_canvas[0] - data->points[i][0], 2) + pow(mouse_pos_in_canvas[1] - data->points[i][1], 2)) < min_dist)
						{
							min_dist = sqrt(pow(mouse_pos_in_canvas[0] - data->points[i][0], 2) + pow(mouse_pos_in_canvas[1] - data->points[i][1], 2));
							data_id = i;
						}
					for (int i = 0; i <= 2 * n - 1; i++)
						if (sqrt(pow(mouse_pos_in_canvas[0] - control_points[i][0], 2) + pow(mouse_pos_in_canvas[1] - control_points[i][1], 2)) < min_dist)
						{
							min_dist = sqrt(pow(mouse_pos_in_canvas[0] - control_points[i][0], 2) + pow(mouse_pos_in_canvas[1] - control_points[i][1], 2));
							control_id = i;
							data_id = -1;
						}
				}
				// 鼠标拖动，选中型值点
				if (data_id > -1)
				{
					float delta_x = mouse_pos_in_canvas[0] - data->points[data_id][0];
					float delta_y = mouse_pos_in_canvas[1] - data->points[data_id][1];
					// 根据拖动位置修改型值点位置
					data->points[data_id] = mouse_pos_in_canvas;
					// 更新两侧切线控制点位置
					if (data_id == 0)
					{
						control_points[0][0] += delta_x;
						control_points[0][1] += delta_y;
					}
					else if (data_id == n)
					{
						control_points[2 * n - 1][0] += delta_x;
						control_points[2 * n - 1][1] += delta_y;
					}
					else
					{
						control_points[2 * data_id - 1][0] += delta_x;
						control_points[2 * data_id - 1][1] += delta_y;
						control_points[2 * data_id][0] += delta_x;
						control_points[2 * data_id][1] += delta_y;
					}
					// 鼠标左键松开
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// 修改结束
						data_id = -1;
				}
				// 鼠标拖动，选中切线控制点
				if (control_id > -1)
				{
					// 根据拖动位置修改切线控制点信息
					control_points[control_id] = mouse_pos_in_canvas;
					// 鼠标左键松开
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// 修改结束
						control_id = -1;
				}
			}

			// chordal参数化
			vector<float> t;
			t.clear();
			if (data->points.size() >= 2)
			{
				float temp = 100.0f;
				int n = data->points.size() - 1;
				t.push_back(0.0f);
				for (int i = 1; i <= n; i++)
					t.push_back(t.back() + sqrt(pow(data->points[i][0] - data->points[i - 1][0], 2) + pow(data->points[i][1] - data->points[i - 1][1], 2)) / temp);
			}

			vector<pointf2> fitting_points;
			fitting_points.clear();
			// 生成C2曲线
			// 先求曲线，再更新切线控制点
			if (e_1 == 0 || e_1 == 1)
			{
				if (data->points.size() == 2)
				{
					// 对参数t均匀采样
					for (float t_iter = t[0]; t_iter <= t[1]; t_iter += 0.01f)
					{
						float x = data->points[1][0] / (t[1] - t[0]) * (t_iter - t[0]) + data->points[0][0] / (t[1] - t[0]) * (t[1] - t_iter);
						float y = data->points[1][1] / (t[1] - t[0]) * (t_iter - t[0]) + data->points[0][1] / (t[1] - t[0]) * (t[1] - t_iter);
						fitting_points.push_back(pointf2(x, y));
					}
					// 更新切线控制点位置
					control_points.clear();
					float x = data->points[1][0] / (t[1] - t[0]) - data->points[0][0] / (t[1] - t[0]) + data->points[0][0];
					float y = data->points[1][1] / (t[1] - t[0]) - data->points[0][1] / (t[1] - t[0]) + data->points[0][1];
					control_points.push_back(pointf2(x, y));
					x = -data->points[1][0] / (t[1] - t[0]) + data->points[0][0] / (t[1] - t[0]) + data->points[1][0];
					y = -data->points[1][1] / (t[1] - t[0]) + data->points[0][1] / (t[1] - t[0]) + data->points[1][1];
					control_points.push_back(pointf2(x, y));
				}
				else if (data->points.size() > 2)
				{
					int n = data->points.size() - 1;
					// 求解线性方程组，得到三弯矩方程的系数
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
					// 对参数t分段均匀采样
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
					// 更新切线控制点位置
					control_points.clear();
					for (int i = 0; i < n; i++)
					{
						if (i == 0)
						{
							float x = -h[i] / 6 * M_x(i, 0) - data->points[i][0] / h[i] + data->points[i + 1][0] / h[i] + data->points[i][0];
							float y = -h[i] / 6 * M_y(i, 0) - data->points[i][1] / h[i] + data->points[i + 1][1] / h[i] + data->points[i][1];
							control_points.push_back(pointf2(x, y));
							x = - h[i] / 3 * M_x(i, 0) + data->points[i][0] / h[i] - data->points[i + 1][0] / h[i] + data->points[i + 1][0];
							y = - h[i] / 3 * M_y(i, 0) + data->points[i][1] / h[i] - data->points[i + 1][1] / h[i] + data->points[i + 1][1];
							control_points.push_back(pointf2(x, y));
						}
						else if (i == n - 1)
						{
							float x = -h[i] / 3 * M_x(i - 1, 0) - data->points[i][0] / h[i] + data->points[i + 1][0] / h[i] + data->points[i][0];
							float y = -h[i] / 3 * M_y(i - 1, 0) - data->points[i][1] / h[i] + data->points[i + 1][1] / h[i] + data->points[i][1];
							control_points.push_back(pointf2(x, y));
							x = -h[i] / 6 * M_x(i - 1, 0) + data->points[i][0] / h[i] - data->points[i + 1][0] / h[i] + data->points[i + 1][0];
							y = -h[i] / 6 * M_y(i - 1, 0) + data->points[i][1] / h[i] - data->points[i + 1][1] / h[i] + data->points[i + 1][1];
							control_points.push_back(pointf2(x, y));
						}
						else
						{
							float x = -h[i] / 3 * M_x(i - 1, 0) - h[i] / 6 * M_x(i, 0) - data->points[i][0] / h[i] + data->points[i + 1][0] / h[i] + data->points[i][0];
							float y = -h[i] / 3 * M_y(i - 1, 0) - h[i] / 6 * M_y(i, 0) - data->points[i][1] / h[i] + data->points[i + 1][1] / h[i] + data->points[i][1];
							control_points.push_back(pointf2(x, y));
							x = -h[i] / 6 * M_x(i - 1, 0) - h[i] / 3 * M_x(i, 0) + data->points[i][0] / h[i] - data->points[i + 1][0] / h[i] + data->points[i + 1][0];
							y = -h[i] / 6 * M_y(i - 1, 0) - h[i] / 3 * M_y(i, 0) + data->points[i][1] / h[i] - data->points[i + 1][1] / h[i] + data->points[i + 1][1];
							control_points.push_back(pointf2(x, y));
						}
					}
				}
			}
			// 生成G1或G0曲线
			// 根据切线控制点求曲线
			else if (e_2 == 1 || e_2 == 0)
			{
			    int n = data->points.size() - 1;
				// 分段单独处理
			    for (int i = 0; i < n; i++)
			    {
					// 已知两端点取值及一阶导数，求三次函数
					MatrixXf A(4, 4);
					A(0, 0) = 1;
					A(0, 1) = t[i];
					A(0, 2) = pow(t[i], 2);
					A(0, 3) = pow(t[i], 3);
					A(1, 0) = 1;
					A(1, 1) = t[i + 1];
					A(1, 2) = pow(t[i + 1], 2);
					A(1, 3) = pow(t[i + 1], 3);
					A(2, 0) = 0;
					A(2, 1) = 1;
					A(2, 2) = 2 * t[i];
					A(2, 3) = 3 * pow(t[i], 2);
					A(3, 0) = 0;
					A(3, 1) = 1;
					A(3, 2) = 2 * t[i + 1];
					A(3, 3) = 3 * pow(t[i + 1], 2);
					MatrixXf V_x(4, 1);
					V_x(0, 0) = data->points[i][0];
					V_x(1, 0) = data->points[i + 1][0];
					V_x(2, 0) = control_points[2 * i][0] - data->points[i][0];
					V_x(3, 0) = -control_points[2 * i + 1][0] + data->points[i + 1][0];
					MatrixXf V_y(4, 1);
					V_y(0, 0) = data->points[i][1];
					V_y(1, 0) = data->points[i + 1][1];
					V_y(2, 0) = control_points[2 * i][1] - data->points[i][1];
					V_y(3, 0) = -control_points[2 * i + 1][1] + data->points[i + 1][1];
					MatrixXf M_x = A.colPivHouseholderQr().solve(V_x);
					MatrixXf M_y = A.colPivHouseholderQr().solve(V_y);
					// 对参数t均匀采样
					for (float t_iter = t[i]; t_iter <= t[i + 1]; t_iter += 0.01f)
					{
						float x = M_x(0, 0) + M_x(1, 0) * t_iter + M_x(2, 0) * pow(t_iter, 2) + M_x(3, 0) * pow(t_iter, 3);
						float y = M_y(0, 0) + M_y(1, 0) * t_iter + M_y(2, 0) * pow(t_iter, 2) + M_y(3, 0) * pow(t_iter, 3);
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
			// 标记输入点集，已选中点标记为蓝色，未选中点标记为黄色
			for (int n = 0; n < data->points.size(); n++)
				if (n == data_id)
					draw_list->AddCircle(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), 3, IM_COL32(0, 0, 255, 255), 0, 2.0f);
				else
					draw_list->AddCircle(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), 3, IM_COL32(255, 255, 0, 255), 0, 2.0f);
			// 连接采样点，绘制拟合函数图像
			for (int n = 0; n + 1 < fitting_points.size(); n++)
				draw_list->AddLine(ImVec2(origin.x + fitting_points[n][0], origin.y + fitting_points[n][1]), ImVec2(origin.x + fitting_points[n + 1][0], origin.y + fitting_points[n + 1][1]), IM_COL32(255, 255, 0, 255), 2.0f);
			// 编辑型值点处切线信息时，标记切线控制点，已选中点标记为蓝色，未选中点标记为绿色
			if (e_1 == 2)
			{
				int n = data->points.size() - 1;
				for (int i = 0; i < n; i++)
				{
					if (2 * i == control_id)
						draw_list->AddCircle(ImVec2(origin.x + control_points[2 * i][0], origin.y + control_points[2 * i][1]), 3, IM_COL32(0, 0, 255, 255), 0, 2.0f);
					else
						draw_list->AddCircle(ImVec2(origin.x + control_points[2 * i][0], origin.y + control_points[2 * i][1]), 3, IM_COL32(0, 255, 0, 255), 0, 2.0f);
					draw_list->AddLine(ImVec2(origin.x + control_points[2 * i][0], origin.y + control_points[2 * i][1]), ImVec2(origin.x + data->points[i][0], origin.y + data->points[i][1]), IM_COL32(0, 255, 0, 255), 2.0f);
					if (2 * i + 1 == control_id)
						draw_list->AddCircle(ImVec2(origin.x + control_points[2 * i + 1][0], origin.y + control_points[2 * i + 1][1]), 3, IM_COL32(0, 0, 255, 255), 0, 2.0f);
					else
						draw_list->AddCircle(ImVec2(origin.x + control_points[2 * i + 1][0], origin.y + control_points[2 * i + 1][1]), 3, IM_COL32(0, 255, 0, 255), 0, 2.0f);
					draw_list->AddLine(ImVec2(origin.x + control_points[2 * i + 1][0], origin.y + control_points[2 * i + 1][1]), ImVec2(origin.x + data->points[i + 1][0], origin.y + data->points[i + 1][1]), IM_COL32(0, 255, 0, 255), 2.0f);
				}
			}
			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}
