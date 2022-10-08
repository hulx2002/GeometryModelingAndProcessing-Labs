#include "DenoiseSystem.h"

#include "../Components/DenoiseData.h"

#include <_deps/imgui/imgui.h>

#include <spdlog/spdlog.h>

#include <Eigen/Sparse>

using namespace Ubpa;
using namespace std;
using namespace Eigen;

void DenoiseSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<DenoiseData>();
		if (!data)
			return;

		if (ImGui::Begin("Denoise")) {
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

			// 生成极小曲面（全局方法）
			if (ImGui::Button("Minimal Surface")) {
				[&]() {
					if (!data->heMesh->IsTriMesh() || data->heMesh->IsEmpty()) {
						spdlog::warn("HEMesh isn't triangle mesh or is empty");
						return;
					}

					const size_t N = data->heMesh->Vertices().size();
					SparseMatrix<float> L(N, N);
					VectorXf delta_x(N);
					VectorXf delta_y(N);
					VectorXf delta_z(N);
					// 遍历顶点，构建稀疏方程组
					for (size_t i = 0; i < N; i++) {
						auto P = data->heMesh->Vertices().at(i);
						// 边界顶点，固定坐标
						if (P->IsOnBoundary()) {
							L.insert(i, i) = 1.f;
							delta_x(i) = P->position[0];
							delta_y(i) = P->position[1];
							delta_z(i) = P->position[2];
						}
						// 非边界顶点，微分坐标=0
						else {
							L.insert(i, i) = 0.f;
							delta_x(i) = 0.f;
							delta_y(i) = 0.f;
							delta_z(i) = 0.f;
							for (auto Q : P->AdjVertices()) {
								size_t j = data->heMesh->Index(Q);
								auto Q_alpha = P->HalfEdgeTo(Q)->Next()->End();
								auto Q_beta = Q->HalfEdgeTo(P)->Next()->End();
								float P_Q = P->position.distance(Q->position);
								float P_Qalpha = P->position.distance(Q_alpha->position);
								float P_Qbeta = P->position.distance(Q_beta->position);
								float Q_Qalpha = Q->position.distance(Q_alpha->position);
								float Q_Qbeta = Q->position.distance(Q_beta->position);
								float alpha = acos((pow(P_Qalpha, 2) + pow(Q_Qalpha, 2) - pow(P_Q, 2)) / (2.f * P_Qalpha * Q_Qalpha));
								float beta = acos((pow(P_Qbeta, 2) + pow(Q_Qbeta, 2) - pow(P_Q, 2)) / (2.f * P_Qbeta * Q_Qbeta));
								L.insert(i, j) = -(1.f / tan(alpha) + 1.f / tan(beta));
								L.coeffRef(i, i) += 1.f / tan(alpha) + 1.f / tan(beta);
							}
						}
					}
					// 求解稀疏方程组
					SparseLU<SparseMatrix<float>> solver;
					solver.compute(L);
					VectorXf v_x = solver.solve(delta_x);
					VectorXf v_y = solver.solve(delta_y);
					VectorXf v_z = solver.solve(delta_z);
					// 更新顶点坐标
					for (size_t i = 0; i < N; i++) {
						data->heMesh->Vertices().at(i)->position[0] = v_x(i);
						data->heMesh->Vertices().at(i)->position[1] = v_y(i);
						data->heMesh->Vertices().at(i)->position[2] = v_z(i);
					}

					spdlog::info("minimal surface success");
				}();
			}

			// 参数化
			if (ImGui::Button("Parameterization")) {
				[&]() {
					if (!data->heMesh->IsTriMesh() || data->heMesh->IsEmpty()) {
						spdlog::warn("HEMesh isn't triangle mesh or is empty");
						return;
					}

					const size_t N = data->heMesh->Vertices().size();
					SparseMatrix<float> L(N, N);
					VectorXf delta_x(N);
					VectorXf delta_y(N);
					// 构建稀疏方程组
					// 边界顶点，映射到正方形边界
					float sum_distance = 0.f;
					for (auto* e_0 : data->heMesh->Boundaries()) {
						auto* e = e_0;
						do {
							sum_distance += e->Origin()->position.distance(e->End()->position);
							e = e->Next();
						} while (e != e_0);
					}
					for (auto* e_0 : data->heMesh->Boundaries()) {
						size_t i = data->heMesh->Index(e_0->Origin());
						float distance = 0.f;
						L.insert(i, i) = 1.f;
						delta_x(i) = 0.f;
						delta_y(i) = 0.f;
						for (auto* e = e_0; e->Next() != e_0; e = e->Next()) {
							i = data->heMesh->Index(e->End());
							distance += e->Origin()->position.distance(e->End()->position);
							L.insert(i, i) = 1.f;
							if (distance / sum_distance <= 0.25f) {
								delta_x(i) = 4.f * distance / sum_distance;
								delta_y(i) = 0.f;
							}
							else if (distance / sum_distance <= 0.5f) {
								delta_x(i) = 1.f;
								delta_y(i) = 4.f * distance / sum_distance - 1.f;
							}
							else if (distance / sum_distance <= 0.75f) {
								delta_x(i) = 3.f - 4.f * distance / sum_distance;
								delta_y(i) = 1.f;
							}
							else {
								delta_x(i) = 0.f;
								delta_y(i) = 4.f - 4.f * distance / sum_distance;
							}
						}
					}
					// 非边界顶点，微分坐标=0
					for (size_t i = 0; i < N; i++) {
						auto P = data->heMesh->Vertices().at(i);
						if (!P->IsOnBoundary()) {
							L.insert(i, i) = 0.f;
							delta_x(i) = 0.f;
							delta_y(i) = 0.f;
							for (auto Q : P->AdjVertices()) {
								size_t j = data->heMesh->Index(Q);
								auto Q_alpha = P->HalfEdgeTo(Q)->Next()->End();
								auto Q_beta = Q->HalfEdgeTo(P)->Next()->End();
								float P_Q = P->position.distance(Q->position);
								float P_Qalpha = P->position.distance(Q_alpha->position);
								float P_Qbeta = P->position.distance(Q_beta->position);
								float Q_Qalpha = Q->position.distance(Q_alpha->position);
								float Q_Qbeta = Q->position.distance(Q_beta->position);
								float alpha = acos((pow(P_Qalpha, 2) + pow(Q_Qalpha, 2) - pow(P_Q, 2)) / (2.f * P_Qalpha * Q_Qalpha));
								float beta = acos((pow(P_Qbeta, 2) + pow(Q_Qbeta, 2) - pow(P_Q, 2)) / (2.f * P_Qbeta * Q_Qbeta));
								L.insert(i, j) = -(1.f / tan(alpha) + 1.f / tan(beta));
								L.coeffRef(i, i) += 1.f / tan(alpha) + 1.f / tan(beta);
							}
						}
					}
					// 求解稀疏方程组
					SparseLU<SparseMatrix<float>> solver;
					solver.compute(L);
					VectorXf v_x = solver.solve(delta_x);
					VectorXf v_y = solver.solve(delta_y);
					// 更新顶点坐标
					for (size_t i = 0; i < N; i++) {
						data->heMesh->Vertices().at(i)->position[0] = v_x(i);
						data->heMesh->Vertices().at(i)->position[1] = v_y(i);
						data->heMesh->Vertices().at(i)->position[2] = 0.f;
					}

					spdlog::info("parameterization success");
				}();
			}
		}
		ImGui::End();
	});
}
