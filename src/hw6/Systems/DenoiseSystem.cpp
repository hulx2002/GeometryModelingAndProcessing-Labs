#include "DenoiseSystem.h"

#include "../Components/DenoiseData.h"

#include <_deps/imgui/imgui.h>

#include <spdlog/spdlog.h>

#define PI acos(-1)

using namespace Ubpa;

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

			static int iterations = 0;
			static float lambda = 0.f;
			ImGui::SliderInt("iterations", &iterations, 0, 1000);
			ImGui::SliderFloat("lambda", &lambda, 0.f, 0.1f, "%.3f");
			if (ImGui::Button("Minimal Surface")) {
				[&]() {
					if (!data->heMesh->IsTriMesh() || data->heMesh->IsEmpty()) {
						spdlog::warn("HEMesh isn't triangle mesh or is empty");
						return;
					}

					const size_t N = data->heMesh->Vertices().size();
					std::vector<Ubpa::pointf3> positions(N);
					for (int count = 0; count < iterations; count++) {
						for (size_t i = 0; i < N; i++) {
							auto P = data->heMesh->Vertices().at(i);
							if (P->IsOnBoundary()) {
								positions[i] = P->position;
							}
							else {
								Ubpa::vecf3 Hn = Ubpa::vecf3{ 0.f };
								float A = 0.f;
								for (auto Q_alpha : P->AdjVertices()) {
									auto Q_beta = P->HalfEdgeTo(Q_alpha)->Next()->End();
									float P_Qalpha = P->position.distance(Q_alpha->position);
									float P_Qbeta = P->position.distance(Q_beta->position);
									float Qalpha_Qbeta = Q_alpha->position.distance(Q_beta->position);
									float alpha = acos((pow(P_Qalpha, 2) + pow(Qalpha_Qbeta, 2) - pow(P_Qbeta, 2)) / (2.f * P_Qalpha * Qalpha_Qbeta));
									float beta = acos((pow(P_Qbeta, 2) + pow(Qalpha_Qbeta, 2) - pow(P_Qalpha, 2)) / (2.f * P_Qbeta * Qalpha_Qbeta));
									Hn += 1.f / tan(alpha) * (P->position - Q_beta->position) + 1.f / tan(beta) * (P->position - Q_alpha->position);
									if (alpha > PI / 2.f || beta > PI / 2.f)
										A += P_Qalpha * P_Qbeta * sin(PI - alpha - beta) / 8.f;
									else if (PI - alpha - beta > PI / 2.f)
										A += P_Qalpha * P_Qbeta * sin(PI - alpha - beta) / 4.f;
									else
										A += (1.f / tan(alpha) * pow(P_Qbeta, 2) + 1.f / tan(beta) * pow(P_Qalpha, 2)) / 8.f;
								}
								Hn /= (2.f * A);
								positions[i] = P->position - lambda * Hn;
							}
						}
						for (size_t i = 0; i < N; i++)
							data->heMesh->Vertices().at(i)->position = positions[i];
					}

					spdlog::info("minimal surface success");
				}();
			}
		}
		ImGui::End();
	});
}
