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

			// ѡ��������ֵ�㡢�޸��϶���ֵ���༭��ֵ�㴦��������Ϣ
			static int e_1 = 0;
			ImGui::RadioButton("Add points", &e_1, 0);
			ImGui::RadioButton("Edit points", &e_1, 1);
			ImGui::RadioButton("Edit tangents", &e_1, 2);
			// �༭��ֵ�㴦������Ϣʱ��ѡ�����߼��������Խ���
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
			// ������ֵ��
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
			// �޸��϶���ֵ��
			else if (e_1 == 1)
			{
				int n = data->points.size() - 1;
				// ���������
				if (is_hovered && data_id == -1 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
				{
					// Ѱ�Ҿ�����λ���������ֵ��
					float min_dist = 3.0f;
					for (int i = 0; i <= n; i++)
						if (sqrt(pow(mouse_pos_in_canvas[0] - data->points[i][0], 2) + pow(mouse_pos_in_canvas[1] - data->points[i][1], 2)) < min_dist)
						{
							min_dist = sqrt(pow(mouse_pos_in_canvas[0] - data->points[i][0], 2) + pow(mouse_pos_in_canvas[1] - data->points[i][1], 2));
							data_id = i;
						}
				}
				// ����϶�
				if (data_id > -1)
				{
					// �����϶�λ���޸���ֵ��λ��
					data->points[data_id] = mouse_pos_in_canvas;
					// �������ɿ�
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// �޸Ľ���
						data_id = -1;
				}
			}
			// �༭��ֵ�㴦������Ϣ������1�׼�������
			else if (e_1 == 2 && e_2 == 1)
			{
				int n = data->points.size() - 1;
				// ���������
				if (is_hovered && data_id == -1 && control_id == -1 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
				{
					// Ѱ�Ҿ�����λ���������ֵ������߿��Ƶ�
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
				// ����϶���ѡ����ֵ��
				if (data_id > -1)
				{
					float delta_x = mouse_pos_in_canvas[0] - data->points[data_id][0];
					float delta_y = mouse_pos_in_canvas[1] - data->points[data_id][1];
					// �����϶�λ���޸���ֵ��λ��
					data->points[data_id] = mouse_pos_in_canvas;
					// �����������߿��Ƶ�λ��
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
					// �������ɿ�
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// �޸Ľ���
						data_id = -1;
				}
				// ����϶���ѡ�����߿��Ƶ�
				if (control_id > -1)
				{
					// �����϶�λ���޸����߿��Ƶ���Ϣ
					control_points[control_id] = mouse_pos_in_canvas;
					// ������һ�����߿��Ƶ���Ϣ��������һ�����ߴ�С�������������߷�������
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
					// �������ɿ�
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// �޸Ľ���
						control_id = -1;
				}
			}
			// �༭��ֵ�㴦������Ϣ������0�׼�������
			else if (e_1 == 2 && e_2 == 0)
			{
				int n = data->points.size() - 1;
				// ���������
				if (is_hovered && data_id == -1 && control_id == -1 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
				{
					// Ѱ�Ҿ�����λ���������ֵ������߿��Ƶ�
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
				// ����϶���ѡ����ֵ��
				if (data_id > -1)
				{
					float delta_x = mouse_pos_in_canvas[0] - data->points[data_id][0];
					float delta_y = mouse_pos_in_canvas[1] - data->points[data_id][1];
					// �����϶�λ���޸���ֵ��λ��
					data->points[data_id] = mouse_pos_in_canvas;
					// �����������߿��Ƶ�λ��
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
					// �������ɿ�
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// �޸Ľ���
						data_id = -1;
				}
				// ����϶���ѡ�����߿��Ƶ�
				if (control_id > -1)
				{
					// �����϶�λ���޸����߿��Ƶ���Ϣ
					control_points[control_id] = mouse_pos_in_canvas;
					// �������ɿ�
					if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
						// �޸Ľ���
						control_id = -1;
				}
			}

			// chordal������
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
			// ����C2����
			// �������ߣ��ٸ������߿��Ƶ�
			if (e_1 == 0 || e_1 == 1)
			{
				if (data->points.size() == 2)
				{
					// �Բ���t���Ȳ���
					for (float t_iter = t[0]; t_iter <= t[1]; t_iter += 0.01f)
					{
						float x = data->points[1][0] / (t[1] - t[0]) * (t_iter - t[0]) + data->points[0][0] / (t[1] - t[0]) * (t[1] - t_iter);
						float y = data->points[1][1] / (t[1] - t[0]) * (t_iter - t[0]) + data->points[0][1] / (t[1] - t[0]) * (t[1] - t_iter);
						fitting_points.push_back(pointf2(x, y));
					}
					// �������߿��Ƶ�λ��
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
					// ������Է����飬�õ�����ط��̵�ϵ��
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
					// �Բ���t�ֶξ��Ȳ���
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
					// �������߿��Ƶ�λ��
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
			// ����G1��G0����
			// �������߿��Ƶ�������
			else if (e_2 == 1 || e_2 == 0)
			{
			    int n = data->points.size() - 1;
				// �ֶε�������
			    for (int i = 0; i < n; i++)
			    {
					// ��֪���˵�ȡֵ��һ�׵����������κ���
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
					// �Բ���t���Ȳ���
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
			// ���ԭ��
			draw_list->AddCircle(ImVec2(origin.x, origin.y), 3, IM_COL32(255, 0, 0, 255), 0, 2.0f);
			// �������㼯����ѡ�е���Ϊ��ɫ��δѡ�е���Ϊ��ɫ
			for (int n = 0; n < data->points.size(); n++)
				if (n == data_id)
					draw_list->AddCircle(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), 3, IM_COL32(0, 0, 255, 255), 0, 2.0f);
				else
					draw_list->AddCircle(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), 3, IM_COL32(255, 255, 0, 255), 0, 2.0f);
			// ���Ӳ����㣬������Ϻ���ͼ��
			for (int n = 0; n + 1 < fitting_points.size(); n++)
				draw_list->AddLine(ImVec2(origin.x + fitting_points[n][0], origin.y + fitting_points[n][1]), ImVec2(origin.x + fitting_points[n + 1][0], origin.y + fitting_points[n + 1][1]), IM_COL32(255, 255, 0, 255), 2.0f);
			// �༭��ֵ�㴦������Ϣʱ��������߿��Ƶ㣬��ѡ�е���Ϊ��ɫ��δѡ�е���Ϊ��ɫ
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
