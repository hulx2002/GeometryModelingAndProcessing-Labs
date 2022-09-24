#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>

using namespace Ubpa;
using namespace std;

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");

			// 选择细分方法
			static int e_1 = 0;
			ImGui::RadioButton("stop subdivision", &e_1, 0);
			ImGui::RadioButton("approach chaikin", &e_1, 1);
			ImGui::RadioButton("approach cubicbspline", &e_1, 2);
			ImGui::RadioButton("interpolation 4points", &e_1, 3);
			// 选择开放曲线或闭合曲线
			static int e_2 = 0;
			ImGui::RadioButton("open curve", &e_2, 0);
			ImGui::RadioButton("closed curve", &e_2, 1);

			// 设置迭代次数
			static int iterations = 0;
			ImGui::SliderInt("iterations", &iterations, 0, 10);
			// 设置参数值
			static float alpha = 0.0f;
			ImGui::SliderFloat("alpha", &alpha, 0.0f, 0.125f, "%.3f");

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

			// 生成细分曲线
			vector<pointf2> fitting_points;
			fitting_points.clear();
			if (e_1 > 0 && data->points.size() >= 2)
			{
				// chaikin方法
				if (e_1 == 1)
				{
					fitting_points = data->points;
					for (int count = 0; count < iterations; count++)
					{
						int n = fitting_points.size();
						vector<pointf2> temp;
						temp.clear();
						// 生成新边点
						for (int i = 0; i < n - 1; i++)
						{
							float x = 3.0f / 4.0f * fitting_points[i][0] + 1.0f / 4.0f * fitting_points[i + 1][0];
							float y = 3.0f / 4.0f * fitting_points[i][1] + 1.0f / 4.0f * fitting_points[i + 1][1];
							temp.push_back(pointf2(x, y));
							x = 1.0f / 4.0f * fitting_points[i][0] + 3.0f / 4.0f * fitting_points[i + 1][0];
							y = 1.0f / 4.0f * fitting_points[i][1] + 3.0f / 4.0f * fitting_points[i + 1][1];
							temp.push_back(pointf2(x, y));
						}
						// 闭合曲线情况
						if (e_2 == 1 && n >= 3)
						{
							float x = 3.0f / 4.0f * fitting_points[n - 1][0] + 1.0f / 4.0f * fitting_points[0][0];
							float y = 3.0f / 4.0f * fitting_points[n - 1][1] + 1.0f / 4.0f * fitting_points[0][1];
							temp.push_back(pointf2(x, y));
							x = 1.0f / 4.0f * fitting_points[n - 1][0] + 3.0f / 4.0f * fitting_points[0][0];
							y = 1.0f / 4.0f * fitting_points[n - 1][1] + 3.0f / 4.0f * fitting_points[0][1];
							temp.push_back(pointf2(x, y));
						}
						fitting_points = temp;
					}
				}
				// 三次B样条细分方法
				else if (e_1 == 2)
				{
					fitting_points = data->points;
					for (int count = 0; count < iterations; count++)
					{
						int n = fitting_points.size();
						vector<pointf2> temp;
						temp.clear();
						// 生成新边点和新点点
						for (int i = 0; i < n - 2; i++)
						{
							float x = 1.0f / 2.0f * fitting_points[i][0] + 1.0f / 2.0f * fitting_points[i + 1][0];
							float y = 1.0f / 2.0f * fitting_points[i][1] + 1.0f / 2.0f * fitting_points[i + 1][1];
							temp.push_back(pointf2(x, y));
							x = 1.0f / 8.0f * fitting_points[i][0] + 3.0f / 4.0f * fitting_points[i + 1][0] + 1.0f / 8.0f * fitting_points[i + 2][0];
							y = 1.0f / 8.0f * fitting_points[i][1] + 3.0f / 4.0f * fitting_points[i + 1][1] + 1.0f / 8.0f * fitting_points[i + 2][1];
							temp.push_back(pointf2(x, y));
						}
						float x = 1.0f / 2.0f * fitting_points[n - 2][0] + 1.0f / 2.0f * fitting_points[n - 1][0];
						float y = 1.0f / 2.0f * fitting_points[n - 2][1] + 1.0f / 2.0f * fitting_points[n - 1][1];
						temp.push_back(pointf2(x, y));
						// 闭合曲线情况
						if (e_2 == 1 && n >= 3)
						{
							x = 1.0f / 8.0f * fitting_points[n - 2][0] + 3.0f / 4.0f * fitting_points[n - 1][0] + 1.0f / 8.0f * fitting_points[0][0];
							y = 1.0f / 8.0f * fitting_points[n - 2][1] + 3.0f / 4.0f * fitting_points[n - 1][1] + 1.0f / 8.0f * fitting_points[0][1];
							temp.push_back(pointf2(x, y));
							x = 1.0f / 2.0f * fitting_points[n - 1][0] + 1.0f / 2.0f * fitting_points[0][0];
							y = 1.0f / 2.0f * fitting_points[n - 1][1] + 1.0f / 2.0f * fitting_points[0][1];
							temp.push_back(pointf2(x, y));
							x = 1.0f / 8.0f * fitting_points[n - 1][0] + 3.0f / 4.0f * fitting_points[0][0] + 1.0f / 8.0f * fitting_points[1][0];
							y = 1.0f / 8.0f * fitting_points[n - 1][1] + 3.0f / 4.0f * fitting_points[0][1] + 1.0f / 8.0f * fitting_points[1][1];
							temp.push_back(pointf2(x, y));
						}
						fitting_points = temp;
					}
				}
				// 4点细分方法
				else if (e_1 == 3)
				{
					fitting_points = data->points;
					for (int count = 0; count < iterations; count++)
					{
						int n = fitting_points.size();
						vector<pointf2> temp;
						temp.clear();
						// 保留原有顶点，并对每条边增加一个新顶点
						temp.push_back(fitting_points[0]);
						if (e_2 == 1 && n >= 4)
						{
							float x = (fitting_points[0][0] + fitting_points[1][0]) / 2.0f + alpha * ((fitting_points[0][0] + fitting_points[1][0]) / 2.0f - (fitting_points[n - 1][0] + fitting_points[2][0]) / 2.0f);
							float y = (fitting_points[0][1] + fitting_points[1][1]) / 2.0f + alpha * ((fitting_points[0][1] + fitting_points[1][1]) / 2.0f - (fitting_points[n - 1][1] + fitting_points[2][1]) / 2.0f);
							temp.push_back(pointf2(x, y));
						}
						for (int i = 1; i < n - 2; i++)
						{
							temp.push_back(fitting_points[i]);
							float x = (fitting_points[i][0] + fitting_points[i + 1][0]) / 2.0f + alpha * ((fitting_points[i][0] + fitting_points[i + 1][0]) / 2.0f - (fitting_points[i - 1][0] + fitting_points[i + 2][0]) / 2.0f);
							float y = (fitting_points[i][1] + fitting_points[i + 1][1]) / 2.0f + alpha * ((fitting_points[i][1] + fitting_points[i + 1][1]) / 2.0f - (fitting_points[i - 1][1] + fitting_points[i + 2][1]) / 2.0f);
							temp.push_back(pointf2(x, y));
						}
						if (n >= 3)
							temp.push_back(fitting_points[n - 2]);
						if (e_2 == 1 && n >= 4)
						{
							float x = (fitting_points[n - 2][0] + fitting_points[n - 1][0]) / 2.0f + alpha * ((fitting_points[n - 2][0] + fitting_points[n - 1][0]) / 2.0f - (fitting_points[n - 3][0] + fitting_points[0][0]) / 2.0f);
							float y = (fitting_points[n - 2][1] + fitting_points[n - 1][1]) / 2.0f + alpha * ((fitting_points[n - 2][1] + fitting_points[n - 1][1]) / 2.0f - (fitting_points[n - 3][1] + fitting_points[0][1]) / 2.0f);
							temp.push_back(pointf2(x, y));
						}
						temp.push_back(fitting_points[n - 1]);
						if (e_2 == 1 && n >= 4)
						{
							float x = (fitting_points[n - 1][0] + fitting_points[0][0]) / 2.0f + alpha * ((fitting_points[n - 1][0] + fitting_points[0][0]) / 2.0f - (fitting_points[n - 2][0] + fitting_points[1][0]) / 2.0f);
							float y = (fitting_points[n - 1][1] + fitting_points[0][1]) / 2.0f + alpha * ((fitting_points[n - 1][1] + fitting_points[0][1]) / 2.0f - (fitting_points[n - 2][1] + fitting_points[1][1]) / 2.0f);
							temp.push_back(pointf2(x, y));
						}
						fitting_points = temp;
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
			int n = data->points.size();
			for (int i = 0; i < n; i++)
				draw_list->AddCircle(ImVec2(origin.x + data->points[i][0], origin.y + data->points[i][1]), 3, IM_COL32(255, 255, 0, 255), 0, 2.0f);
			// 绘制细分曲线
			n = fitting_points.size();
			for (int i = 0; i < n - 1; i++)
				draw_list->AddLine(ImVec2(origin.x + fitting_points[i][0], origin.y + fitting_points[i][1]), ImVec2(origin.x + fitting_points[i + 1][0], origin.y + fitting_points[i + 1][1]), IM_COL32(255, 255, 0, 255), 2.0f);
			// 闭合曲线情况
			if (e_2 == 1 && n >= 3)
				draw_list->AddLine(ImVec2(origin.x + fitting_points[n - 1][0], origin.y + fitting_points[n - 1][1]), ImVec2(origin.x + fitting_points[0][0], origin.y + fitting_points[0][1]), IM_COL32(255, 255, 0, 255), 2.0f);
			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}
