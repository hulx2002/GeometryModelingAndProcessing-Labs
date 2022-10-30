#include "DenoiseSystem.h"

#include "../Components/DenoiseData.h"

#include <_deps/imgui/imgui.h>

#include <spdlog/spdlog.h>

#include <Eigen/Dense>

using namespace Ubpa;
using namespace std;
using namespace Eigen;

void DenoiseSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<DenoiseData>();
		if (!data)
			return;

		if (ImGui::Begin("Denoise")) {
			// 输入网格顶点数
			static int n = 0;
			ImGui::InputInt("n", &n);

			if (ImGui::Button("Mesh to HEMesh")) {
				data->heMesh->Clear();
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					if (data->mesh->GetSubMeshes().size() != 1) {
						spdlog::warn("number of submeshes isn't 1");
						return;
					}

					data->copy = *data->mesh;

					std::vector<size_t> indices(data->mesh->GetIndices().begin(), data->mesh->GetIndices().end());
					data->heMesh->Init(indices, 3);
					if (!data->heMesh->IsTriMesh())
						spdlog::warn("HEMesh init fail");

					for (size_t i = 0; i < data->mesh->GetPositions().size(); i++)
						data->heMesh->Vertices().at(i)->position = data->mesh->GetPositions().at(i);

					n = data->heMesh->Vertices().size();

					spdlog::info("Mesh to HEMesh success");
				}();
			}

			if (ImGui::Button("Add Noise")) {
				[&]() {
					if (!data->heMesh->IsTriMesh()) {
						spdlog::warn("HEMesh isn't triangle mesh");
						return;
					}

					for (auto* v : data->heMesh->Vertices()) {
						v->position += data->randomScale * (
							2.f * Ubpa::vecf3{ Ubpa::rand01<float>(),Ubpa::rand01<float>() ,Ubpa::rand01<float>() } - Ubpa::vecf3{ 1.f }
						);
					}

					spdlog::info("Add noise success");
				}();
			}

			if (ImGui::Button("Set Normal to Color")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					data->mesh->SetToEditable();
					const auto& normals = data->mesh->GetNormals();
					std::vector<rgbf> colors;
					for (const auto& n : normals)
						colors.push_back((n.as<valf3>() + valf3{ 1.f }) / 2.f);
					data->mesh->SetColors(std::move(colors));

					spdlog::info("Set Normal to Color Success");
				}();
			}

			if (ImGui::Button("HEMesh to Mesh")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					if (!data->heMesh->IsTriMesh() || data->heMesh->IsEmpty()) {
						spdlog::warn("HEMesh isn't triangle mesh or is empty");
						return;
					}

					data->mesh->SetToEditable();

					const size_t N = data->heMesh->Vertices().size();
					const size_t M = data->heMesh->Polygons().size();
					std::vector<Ubpa::pointf3> positions(N);
					std::vector<uint32_t> indices(M * 3);
					for (size_t i = 0; i < N; i++)
						positions[i] = data->heMesh->Vertices().at(i)->position;
					for (size_t i = 0; i < M; i++) {
						auto tri = data->heMesh->Indices(data->heMesh->Polygons().at(i));
						indices[3 * i + 0] = static_cast<uint32_t>(tri[0]);
						indices[3 * i + 1] = static_cast<uint32_t>(tri[1]);
						indices[3 * i + 2] = static_cast<uint32_t>(tri[2]);
					}
					data->mesh->SetColors({});
					data->mesh->SetUV({});
					data->mesh->SetPositions(std::move(positions));
					data->mesh->SetIndices(std::move(indices));
					data->mesh->SetSubMeshCount(1);
					data->mesh->SetSubMesh(0, { 0, M * 3 });
					data->mesh->GenUV();
					data->mesh->GenNormals();
					data->mesh->GenTangents();

					spdlog::info("HEMesh to Mesh success");
				}();
			}

			if (ImGui::Button("Recover Mesh")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}
					if (data->copy.GetPositions().empty()) {
						spdlog::warn("copied mesh is empty");
						return;
					}

					*data->mesh = data->copy;

					spdlog::info("recover success");
				}();
			}

			if (ImGui::Button("Surface Simplification")) {
				[&]() {
					if (!data->heMesh->IsTriMesh() || data->heMesh->IsEmpty()) {
						spdlog::warn("HEMesh isn't triangle mesh or is empty");
						return;
					}

					// 计算所有初始顶点的Q矩阵
					vector<Matrix4f> Qs;
					// 遍历顶点
					for (size_t i = 0; i < data->heMesh->Vertices().size(); i++) {
						auto* vi = data->heMesh->Vertices().at(i);
						Matrix4f Q = Matrix4f::Zero();
						// 遍历顶点的邻接平面
						for (auto* vj : vi->AdjVertices()) {
							auto* vk = vi->HalfEdgeTo(vj)->Next()->End();
							// 求解邻接平面方程的系数
							Matrix4f xyz_temp;
							xyz_temp(0, 0) = vi->position[0];
							xyz_temp(0, 1) = vi->position[1];
							xyz_temp(0, 2) = vi->position[2];
							xyz_temp(0, 3) = 1.f;
							xyz_temp(1, 0) = vj->position[0];
							xyz_temp(1, 1) = vj->position[1];
							xyz_temp(1, 2) = vj->position[2];
							xyz_temp(1, 3) = 1.f;
							xyz_temp(2, 0) = vk->position[0];
							xyz_temp(2, 1) = vk->position[1];
							xyz_temp(2, 2) = vk->position[2];
							xyz_temp(2, 3) = 1.f;
							xyz_temp(3, 0) = xyz_temp(3, 1) = xyz_temp(3, 2) = 0.f;
							xyz_temp(3, 3) = 1.f;
							Vector4f f_temp;
							f_temp(0) = f_temp(1) = f_temp(2) = 0.f;
							f_temp(3) = 1.f;
							Vector4f abcd_temp = xyz_temp.inverse() * f_temp;
							float a = abcd_temp(0) / sqrt(pow(abcd_temp(0), 2) + pow(abcd_temp(1), 2) + pow(abcd_temp(2), 2));
							float b = abcd_temp(1) / sqrt(pow(abcd_temp(0), 2) + pow(abcd_temp(1), 2) + pow(abcd_temp(2), 2));
							float c = abcd_temp(2) / sqrt(pow(abcd_temp(0), 2) + pow(abcd_temp(1), 2) + pow(abcd_temp(2), 2));
							float d = 1.f / sqrt(pow(abcd_temp(0), 2) + pow(abcd_temp(1), 2) + pow(abcd_temp(2), 2));
							// 更新顶点的Q矩阵
							Q(0, 0) += a * a;
							Q(0, 1) += a * b;
							Q(0, 2) += a * c;
							Q(0, 3) += a * d;
							Q(1, 0) += a * b;
							Q(1, 1) += b * b;
							Q(1, 2) += b * c;
							Q(1, 3) += b * d;
							Q(2, 0) += a * c;
							Q(2, 1) += b * c;
							Q(2, 2) += c * c;
							Q(2, 3) += c * d;
							Q(3, 0) += a * d;
							Q(3, 1) += b * d;
							Q(3, 2) += c * d;
							Q(3, 3) += d * d;
						}
						Qs.push_back(Q);
					}
					while (data->heMesh->Vertices().size() > n) {
						Edge* e_selected;
						float error_min = -1.f;
						Vector4f v_selected;
						// 选择所有有效点对
						for (auto* e : data->heMesh->Edges()) {
							auto* vi = e->HalfEdge()->Origin();
							auto* vj = e->HalfEdge()->End();
							size_t i = data->heMesh->Index(vi);
							size_t j = data->heMesh->Index(vj);
							// 计算最优收缩目标点
							Matrix4f Q = Qs[i] + Qs[j];
							Matrix4f Q_temp = Q;
							Q_temp(3, 0) = Q_temp(3, 1) = Q_temp(3, 2) = 0.f;
							Q_temp(3, 3) = 1.f;
							Vector4f df_temp;
							df_temp(0) = df_temp(1) = df_temp(2) = 0.f;
							df_temp(3) = 1.f;
							Vector4f vi_temp;
							vi_temp(0) = vi->position[0];
							vi_temp(1) = vi->position[1];
							vi_temp(2) = vi->position[2];
							vi_temp(3) = 1.f;
							Vector4f vj_temp;
							vj_temp(0) = vj->position[0];
							vj_temp(1) = vj->position[1];
							vj_temp(2) = vj->position[2];
							vj_temp(3) = 1.f;
							Vector4f v_temp;
							if(Q_temp.determinant() != 0.f)
								v_temp = Q_temp.inverse() * df_temp;
							else if ((vj_temp.transpose() - vi_temp.transpose()) * Q * (vj_temp - vi_temp) != 0.f) {
								float lambda = (vj_temp.transpose() - vi_temp.transpose()) * Q * vj_temp;
								lambda /= (vj_temp.transpose() - vi_temp.transpose()) * Q * (vj_temp - vi_temp);
								v_temp = lambda * vi_temp + (1 - lambda) * vj_temp;
							}
							else
								v_temp = (vi_temp + vj_temp) / 2.f;
							// 计算收缩点对的代价
							float error = v_temp.transpose() * Q * v_temp;
							// 选择代价最小的点对
							if (error_min == -1.f || error < error_min) {
								e_selected = e;
								error_min = error;
								v_selected = v_temp;
							}
						}
						auto* vi = e_selected->HalfEdge()->Origin();
						auto* vj = e_selected->HalfEdge()->End();
						size_t i = data->heMesh->Index(vi);
						size_t j = data->heMesh->Index(vj);
						// 收缩点对，并更新收缩目标点的位置
						data->heMesh->CollapseEdge(e_selected);
						if (i < data->heMesh->Vertices().size()) {
							data->heMesh->Vertices().at(i)->position[0] = v_selected(0);
							data->heMesh->Vertices().at(i)->position[1] = v_selected(1);
							data->heMesh->Vertices().at(i)->position[2] = v_selected(2);
						}
						else {
							data->heMesh->Vertices().at(j)->position[0] = v_selected(0);
							data->heMesh->Vertices().at(j)->position[1] = v_selected(1);
							data->heMesh->Vertices().at(j)->position[2] = v_selected(2);
						}
						// 更新收缩目标点的Q矩阵
						Matrix4f Q = Qs[i] + Qs[j];
						Qs[i] = Q;
						Qs[j] = Qs.back();
						Qs.pop_back();
					}

					spdlog::info("simplification success");
				}();
			}
		}
		ImGui::End();
	});
}