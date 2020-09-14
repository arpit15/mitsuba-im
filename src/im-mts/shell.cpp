#include "shell.h"
#include <mitsuba/render/scene.h>
#include <mitsuba/render/integrator2.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <imgui.h>
#include <examples/imgui_impl_sdl.h>
#include "imgui_impl_opengl2.h"
#include <tinyfiledialogs.h>
#include <cstdio>
#include <memory>

struct InteractiveTransform {
	mitsuba::Transform trafo;

	mitsuba::Vector3 rotUp = mitsuba::Vector3(0, 1, 0);

	float sensitivity = 1;
	float speed = 1;

	InteractiveTransform(const mitsuba::Transform& tx)
		: trafo(tx) {
		mitsuba::Vector4 approxUp = trafo( mitsuba::Vector4(0, 1, 0, 0) );
		if (std::abs(approxUp.x * rotUp.x + approxUp.y * rotUp.y + approxUp.z * rotUp.z) < 0.5f) {
			rotUp = mitsuba::Vector3(0, 0, 1);
		}
		if (std::abs(approxUp.x * rotUp.x + approxUp.y * rotUp.y + approxUp.z * rotUp.z) < 0.5f) {
			rotUp = mitsuba::Vector3(1, 0, 0);
		}
		if (approxUp.x * rotUp.x + approxUp.y * rotUp.y + approxUp.z * rotUp.z < 0.0f) {
			rotUp = -rotUp;
		}
	}

	bool update(ImGuiIO& io) {
		mitsuba::Transform trafo = this->trafo;
		bool changed = false;
		// mouse input
		if (!io.WantCaptureMouse && io.MouseDown[0] && io.MouseDownDuration[0] > 0) {
			float rotY = io.MouseDelta.x * sensitivity / 4;
			float rotX = io.MouseDelta.y * sensitivity / 3;

			if (rotX)
				trafo = trafo * trafo.rotate( mitsuba::Vector3(1, 0, 0), rotX );
			if (rotY)
				trafo = trafo * trafo.rotate( trafo.inverse()(rotUp), -rotY );

			changed = rotY || rotX;
		}
		// keyboard input
		if (!io.WantCaptureKeyboard) {
			float dx = this->speed * io.DeltaTime;

			if (io.KeysDown[SDL_SCANCODE_W]) {
				trafo = trafo * trafo.translate( mitsuba::Vector3(0, 0, 1) * dx );
				changed = true;
			}
			if (io.KeysDown[SDL_SCANCODE_S]) {
				trafo = trafo * trafo.translate( mitsuba::Vector3(0, 0, -1) * dx );
				changed = true;
			}
			if (io.KeysDown[SDL_SCANCODE_A]) {
				trafo = trafo * trafo.translate( mitsuba::Vector3(1, 0, 0) * dx);
				changed = true;
			}
			if (io.KeysDown[SDL_SCANCODE_D]) {
				trafo = trafo * trafo.translate( mitsuba::Vector3(-1, 0, 0) * dx );
				changed = true;
			}
			if (io.KeysDown[SDL_SCANCODE_LSHIFT] || io.KeysDown[SDL_SCANCODE_Q]) {
				trafo = trafo * trafo.translate( mitsuba::Vector3(0, -1, 0) * dx );
				changed = true;
			}
			if (io.KeysDown[SDL_SCANCODE_SPACE] || io.KeysDown[SDL_SCANCODE_E]) {
				trafo = trafo * trafo.translate( mitsuba::Vector3(0, 1, 0) * dx );
				changed = true;
			}
		}
		this->trafo = trafo;
		return changed;
	}
};
struct InteractiveSensor : InteractiveTransform {
	mitsuba::ref<mitsuba::Sensor> sensor;

	InteractiveSensor(mitsuba::ref<mitsuba::Sensor> sensor)
		: InteractiveTransform(sensor->getWorldTransform()->eval(0.5f))
		, sensor(sensor) {
	}
	bool update(ImGuiIO& io) {
		bool changed = InteractiveTransform::update(io);
		if (changed)
			transform(trafo);
		return changed;
	}
	void transform(mitsuba::Transform const& newTf) {
		this->trafo = newTf;
		sensor->setWorldTransform(new mitsuba::AnimatedTransform(trafo));
	}
	void applyTo(mitsuba::Sensor* target) const {
		target->setWorldTransform(new mitsuba::AnimatedTransform(trafo));
	}
};

struct Config {
	ProcessConfig process;
};

struct Document {
	fs::pathstr filePath = fs::pathstr("../mitsuba/scenes/bitterli/living-room/livingroom_PT.xml");
	unsigned long long fileTime = 0;
	std::unique_ptr<Scene> scene{ Scene::load(this->filePath) };
	InteractiveSensor camera = { Scene::cloneSensor(*this->scene->scene->getSensor()) };

	struct Renderer : WorkLane::Worker, WorkLane::Sync {
		struct Integration {
			std::unique_ptr<InteractiveSceneProcess> process;
			std::vector<double> samples;
			std::unique_ptr<StackedPreview> preview;
			float exposureMultiplier[4];
			unsigned long long baseTime = ~0;

			Integration() { }

			Integration(mitsuba::Scene* scene, ProcessConfig const& config) {
				process.reset( process->create(scene, scene->getSampler(), scene->getIntegrator(), config) );
				samples.resize(process->maxThreads);
				preview.reset( preview->create(process->resolution.x, process->resolution.y, process->maxThreads, process->uniqueTargets) );
			}

			void switchFrame() {
				preview->nextGeneration();
				memset(samples.data(), 0, sizeof(samples[0]) * samples.size());
			}

			void runFrame(mitsuba::Sensor* sensor, InteractiveSceneProcess::Controls controls) {
				baseTime = SDL_GetTicks();
				preview->runGeneration( baseTime );
				process->render(sensor, samples.data(), controls);

				int waitCounter = 0;
				while (!preview->upToDate((float const* const*) process->imageData, samples.data(), (int) samples.size()) && waitCounter < 160)
					SDL_Delay(waitCounter += std::min(std::max(waitCounter, 5), 16));
			}

			void updatePreview() {
				preview->update(SDL_GetTicks(), (float const* const*) process->imageData, samples.data(), (int) samples.size());
			}

			double timeSeconds() const {
				unsigned long long ticks = SDL_GetTicks();
				if (ticks > baseTime)
					return double(ticks - baseTime) / 1000.0f;
				return 0.0f;
			}
		};

		mitsuba::Scene* scene;
		InteractiveSensor* sensor;
		Integration integration;
		volatile int restart;
		InteractiveSceneProcess::Controls controls = { };
		bool skipInit, reconfig;

		std::unique_ptr<SceneConfigurator::Changes> pendingChanges;
		ProcessConfig nextConfig;

		Renderer(mitsuba::Scene* scene, InteractiveSensor* sensor, Config const& config)
			: scene(scene)
			, sensor(sensor)
			, restart(false)
			, skipInit(false) {
			reconfigure(config.process);
		}

		void reconfigure(ProcessConfig const& config) {
			nextConfig = config;
			reconfig = true;
		}

		void recreateIntegration(Config const* config = nullptr) {
			if (config) {
				nextConfig = config->process;
				reconfig = false;
			}
			integration = Integration(scene, nextConfig);
			reconfig = false;
			// default to interactive
			integration.preview->readyMS = 40;
			integration.preview->updateMS = 80;
		}

		bool applySceneChanges() {
			if (!pendingChanges)
				return false;

			auto oldInt = scene->getIntegrator();
			auto oldFilm = scene->getFilm();
			pendingChanges->apply(scene);
			if (oldInt != scene->getIntegrator() || oldFilm != scene->getFilm())
				reconfig = true;

			pendingChanges.reset();
			return true;
		}

		bool needsSync() const {
			return reconfig || pendingChanges;
		}

		int sync() override {
			bool changes = false;
			changes |= applySceneChanges();
			if (reconfig) {
				recreateIntegration();
				changes = true;
			}
			return changes;
		}

		void work(WorkLane* lane) override {
			bool isRestart = restart;
			restart = false;

			while (needsSync()) {
				lane->synchronize();
				// wake up due to quit/abort?
				if (!lane->continu)
					return;
			}

			sensor->applyTo(scene->getSensor());

			scene->setIntegratorPreprocessed(true);
			scene->preprocess(nullptr, nullptr, -1, -1, -1); // todo: this might crash for more advanced subsurf integrators ...?
			if (!isRestart || !skipInit)
				integration.process->integrator->preprocess(scene, scene->getSensor(), scene->getSampler());
			
			integration.switchFrame();
			controls.continu = &lane->continu;
			controls.abort = &restart;
			
			integration.runFrame(scene->getSensor(), controls);
		}
		
		void quit(WorkLane* lane) override {
			integration.process->pause(false);
		}
	} renderer;

	struct ClassicRenderer {
		mitsuba::Scene* interactiveScene;
		mitsuba::ref<mitsuba::Scene> processedScene;
		InteractiveSensor* sensor;
		std::unique_ptr<SceneProcess> process;
		int revision;
		std::unique_ptr<ImagePreview> preview;
	
		ClassicRenderer(mitsuba::Scene* scene, InteractiveSensor* sensor)
			: interactiveScene(scene)
			, sensor(sensor)
			, revision(0) {
		}

		void reallocate() {
			// todo: assert that scene is actually preprocessed, somehow?
			processedScene = Scene::clonePreprocessed(*interactiveScene);
			process.reset( process->create(processedScene) );
			preview.reset( preview->create(process->resolution.x, process->resolution.y) );
		}

		bool start() {
			if (process) {
				if (process->paused() && !process->cancelled()) {
					process->pause(false);
					return false;
				}
				process->cancel();
			}
			reallocate();
			//sensor->applyTo(processedScene->getSensor());
			process->renderAsync(&revision);
			return true;
		}
		bool stop() {
			if (process) {
				if (!process->paused()) {
					process->pause(true);
					return false;
				}
				else {
					process->cancel();
				}
			}
			return true;
		}
		bool running() const {
			return process && process->running();
		}

		void updatePreview() {
			if (preview && process)
				preview->update((float const*) process->imageData, &revision);
		}
	} classic = { scene->scene, &camera };

	std::unique_ptr<WorkLane> workLane;
	bool autoPaused = false;

	std::unique_ptr<SceneConfigurator> configurator;

	Document(fs::pathstr const& file, Config const& config)
		: filePath(file)
		, fileTime(fs::mts_fs_util::last_write_time(file))
		, renderer(scene->scene, &camera, config) {
	}
	bool fileChanged() const {
		return fs::mts_fs_util::last_write_time(filePath) > fileTime;
	}

	void run() {
		if (!this->workLane) {
			this->renderer.sync();
			this->workLane.reset( WorkLane::create(&this->renderer) );
		}
		this->autoPause(false);
	}
	void pause(bool pause) {
		if (!pause && classic.running())
			classic.process->pause(true);
		if (auto* proc = this->renderer.integration.process.get())
			proc->pause(pause);
	}
	bool paused() const {
		if (auto* proc = this->renderer.integration.process.get())
			return proc->paused;
		else
			return false;
	}
	void restart() {
		if (this->workLane) {
			this->renderer.restart = true;
			pause(false);
		}
	}
	void updatePreview() {
		this->renderer.integration.updatePreview();
		this->classic.updatePreview();
	}

	void autoPause(bool pause) {
		if (pause) {
			if (!this->paused()) {
				this->pause(true);
				autoPaused = true;
			}
		}
		else {
			if (autoPaused) {
				this->pause(false);
				autoPaused = false;
			}
		}
	}

	bool startClassic() {
		this->pause(true);
		return classic.start();
	}
	bool stopClassic() {
		return classic.stop();
	}

	void startConfigurator() {
		configurator.reset( SceneConfigurator::create(scene->scene) );
	}

	void prepareFrame() {
		if (this->workLane) {
			this->workLane->synchronized(this->renderer);
		}
	}
};

struct Session {
	std::vector< std::unique_ptr<Document> > scenes;

	void prepareFrame() {
		for (auto& s : scenes)
			s->prepareFrame();
	}
	void reconfigure(Config const& _cfg) {
		int numScenes = (int) scenes.size();
		if (!numScenes)
			return;

		ProcessConfig cfg = ProcessConfig::resolveDefaults(_cfg.process);
		int minThreads = (cfg.maxThreads) / numScenes;
		int maxThreads = (cfg.maxThreads + (numScenes - 1)) / numScenes;

		int blockedThreads = 0;
		int remainingScenes = numScenes;
		for (auto& s : scenes) {
			ProcessConfig scfg = cfg;
			scfg.maxThreads = maxThreads;
			--remainingScenes;

			if (blockedThreads + scfg.maxThreads + minThreads * remainingScenes > cfg.maxThreads)
				scfg.maxThreads = (cfg.maxThreads - blockedThreads) / (remainingScenes + 1);

			s->renderer.reconfigure(scfg);
			blockedThreads += scfg.maxThreads;
		}
	}

	void run() {
		for (auto& s : scenes)
			s->run();
	}
	void pause(bool pause) {
		for (auto& s : scenes)
			s->pause(pause);
	}
	bool paused() const {
		for (auto& s : scenes)
			if (!s->paused())
				return false;
		return true;
	}
	void restart() {
		for (auto& s : scenes)
			s->restart();
	}

	void autoPause(bool pause) {
		for (auto& s : scenes)
			s->autoPause(pause);
	}
	struct AutoPause {
		Session* session;
		AutoPause(Session* session) : session(session) { if (session) session->autoPause(true); }
		~AutoPause() { if (session) session->autoPause(false); }
	};

	void startClassic() {
		for (auto& s : scenes)
			s->startClassic();
	}
	void stopClassic() {
		for (auto& s : scenes)
			s->stopClassic();
	}

	char const* name() const {
		return scenes.empty() ? "<empty>" : scenes.front()->filePath.s.c_str();
	}
};

std::unique_ptr<Document> tryOpenScene(fs::pathstr path, Config const& config) {
	std::unique_ptr<Document> doc;
	try {
		doc.reset( new Document(path, config) );
	}
	catch (std::exception const& e) {
		tinyfd_messageBox("Could not load scene file!", e.what(), "ok", "error", 1);
	}
	catch (...) {
		tinyfd_messageBox("Error", "Could not load scene file!", "ok", "error", 1);
	}
	return doc;
}

std::unique_ptr<Document> browseForScene(Config const& config) {
	char const* result = tinyfd_openFileDialog("Select scene", 0, 0, 0, "Scenes", false);
	if (result)
		return tryOpenScene(fs::pathstr(result), config);
	return {};
}

void run(int argc, char** argv, SDL_Window* window, SDL_GLContext gl_context, ImGuiContext* ui_context) {
	Config config;
	std::vector< std::unique_ptr<Session> > sessions;
	Session* session = nullptr;

	// initial session, if args passed
	auto openSession = [&sessions, session](std::unique_ptr<Document> newDoc) -> Session* {
		sessions.push_back( std::make_unique<Session>() );
		sessions.back()->scenes.push_back( std::move(newDoc) );
		return sessions.back().get();
	};
	for (int i = 1; i < argc; ++i) {
		auto doc = tryOpenScene(fs::pathstr(argv[i]), config);
		if (doc) {
			if (!session)
				session = openSession(std::move(doc));
			else
				session->scenes.push_back(std::move(doc));
		}
	}
	if (session) {
		if (session->scenes.size() > 1)
			session->reconfigure(config);
		session->run();
	}

	bool show_ui = true;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	clear_color = ImVec4(0.09f, 0.11f, 0.12f, 1.00f);
	float exposure = 1;
	float comparison_factor = 1;
	bool show_diff = false, diff_flip = false;
	int scene_rotation_offset = 0;
	bool alpha_transparent = false;
	int subres_levels = 3;
	bool show_final_render = false;
	bool sync_cams = true;

	bool window_hidden = false;
	bool was_window_hidden = window_hidden;
	auto updateWindowVisibility = [window, &was_window_hidden, &window_hidden]() {
		window_hidden = !(SDL_GetWindowFlags(window) & SDL_WINDOW_SHOWN) || (SDL_GetWindowFlags(window) & (SDL_WINDOW_HIDDEN | SDL_WINDOW_MINIMIZED));
		if (window_hidden != was_window_hidden)
			std::cout << "Window visibility was " << !was_window_hidden << ", now " << !window_hidden << std::endl;
		was_window_hidden = window_hidden;
	};

	bool track_file_changes = true;
	unsigned long long lastTrackerTicks = SDL_GetTicks();

	// Main loop
	while (true) {
		ImGui::SetCurrentContext(ui_context);
		ImGuiIO& io = ImGui::GetIO();

		// Poll and handle events (inputs, window resize, etc.)
		// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
		// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
		// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
		// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
		if (window_hidden)
			updateWindowVisibility();
		bool null_render = false;
		SDL_Event event = { }, nextEvent = { };
		bool had_localized_event = false;
		while (window_hidden ? SDL_WaitEvent(&event) : SDL_PollEvent(&event)) {
			ImGui_ImplSDL2_ProcessEvent(&event);
			if (event.type == SDL_QUIT)
				break;
			// always make sure window not silently revealed, react to events to see if hidden
			if (window_hidden || event.type == SDL_DISPLAYEVENT || event.type == SDL_WINDOWEVENT) {
				updateWindowVisibility();
			}
			// things that can depend on mouse position or state
			had_localized_event |= (event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_MOUSEBUTTONUP || event.type == SDL_MOUSEWHEEL);
			had_localized_event |= (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP);
			if (had_localized_event) {
				if (1 == SDL_PeepEvents(&nextEvent, 1, SDL_PEEKEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT)) {
					// things that change meaning of previous state
					if (nextEvent.type == SDL_MOUSEMOTION || nextEvent.type == SDL_MOUSEBUTTONDOWN || nextEvent.type == SDL_MOUSEBUTTONUP) {
						null_render = true; // fast round
						break;
					}
					// todo: maybe record any key changes?
				}
				// no more events (that we were able to analyze), handle events so far
				else
					break;
			}
		}
		if (event.type == SDL_QUIT)
			break;

		// apply configuration & scene changes
		if (session) {
			session->prepareFrame();
		}

		// GL compositing
		if (session) {
			for (auto& s : session->scenes) {
				s->renderer.integration.preview->maxSubresLevels = subres_levels;
				s->updatePreview();
			}
		}

		// Start the Dear ImGui frame
		if (!null_render)
			ImGui_ImplOpenGL2_NewFrame();
		ImGui_ImplSDL2_NewFrame(window);
		ImGui::NewFrame();

		const unsigned long long currentTicks = SDL_GetTicks();
		bool isTrackingFrame = currentTicks >= lastTrackerTicks + 1500;
		if (isTrackingFrame)
			lastTrackerTicks = currentTicks;

		int mouseSceneIdx = -1;
		if (session && !null_render) {
			int cols = (int) std::ceil( std::sqrt( (float) session->scenes.size() ) );
			int rows = ((int) session->scenes.size() + (cols - 1)) / cols;
			if (show_diff) {
				cols = rows = 1;
			}

			for (int i = 0, ie = (int) session->scenes.size(); i < ie; ++i) {
				int sceneIdx = (i + scene_rotation_offset) % ie;
				auto& s = session->scenes[sceneIdx];
				int col = i % cols;
				int row = (i / cols) % rows;

				int cx = col * int(io.DisplaySize.x) / cols + 1, cy = row * int(io.DisplaySize.y) / rows + 1;
				int cxe = (col+1) * int(io.DisplaySize.x) / cols - 1, cye = (row+1) * int(io.DisplaySize.y) / rows - 1;

				if (io.MousePos.x >= cx && io.MousePos.y >= cy)
					mouseSceneIdx = sceneIdx;

				bool finalPreview = (show_final_render && s->classic.preview);
				Preview& preview = finalPreview ? (Preview&) *s->classic.preview : (Preview&) *s->renderer.integration.preview;

				// Normalize
				if (finalPreview) {
					for (int j = 0; j < 3; ++j)
						s->renderer.integration.exposureMultiplier[j] = exposure;
					s->renderer.integration.exposureMultiplier[3] = (alpha_transparent) ? 1.0f : 0.0f;
				}
				else {
					float clampedSpp = std::max(preview.avgSamples
						, std::min(s->renderer.integration.preview->minSppClamp, (float) s->renderer.integration.process->integrator->getLowerSampleBound()));
					for (int j = 0; j < 3; ++j)
						s->renderer.integration.exposureMultiplier[j] = exposure / clampedSpp;
					s->renderer.integration.exposureMultiplier[3] = (alpha_transparent) ? 1.0f / clampedSpp : 0.0f;

					if (show_diff && (i & 1)) {
						for (int j = 0; j < 3; ++j)
							s->renderer.integration.exposureMultiplier[j] *= -1.0f;
						s->renderer.integration.exposureMultiplier[3] *= -1.0f;
					}

					if (sceneIdx & 1) {
						for (int j = 0; j < 3; ++j)
							s->renderer.integration.exposureMultiplier[j] *= comparison_factor;
					}
				}
				ImGui::GetBackgroundDrawList()->AddCallback(ImDrawCallback_Exposure, s->renderer.integration.exposureMultiplier);
				if (!alpha_transparent && !show_diff)
					ImGui::GetBackgroundDrawList()->AddCallback(ImDrawCallback_NoBlending, 0);

				ImVec2 uv = ImVec2(0, 0), uve = ImVec2(1, 1);
				int ix = (cxe - cx - preview.resX) / 2 + cx, ixe = ix + preview.resX;
				int iy = (cye - cy - preview.resY) / 2 + cy, iye = iy + preview.resY;
				if (ix < cx) {
					uv.x += float(cx - ix) / float(preview.resX);
					ix = cx;
				}
				if (iy < cy) {
					uv.y += float(cy - iy) / float(preview.resY);
					iy = cy;
				}
				if (ixe > cxe) {
					uve.x -= float(ixe - cxe) / float(preview.resX);
					ixe = cxe;
				}
				if (iye > cye) {
					uve.y -= float(iye - cye) / float(preview.resY);
					iye = cye;
				}

				// Show final image
				ImVec2 previewOffset = ImVec2((io.DisplaySize.x - preview.resX) / 2, (io.DisplaySize.y - preview.resY) / 2);
				ImVec2 previewOffsetEnd = ImVec2(previewOffset.x + (float) preview.resX, previewOffset.y + (float) preview.resY);
				ImGui::GetBackgroundDrawList()->AddImage((ImTextureID) preview.previewImg, ImVec2((float) ix, (float) iy), ImVec2((float) ixe, (float) iye), uv, uve);
				// Reset
				ImGui::GetBackgroundDrawList()->AddCallback(ImDrawCallback_ResetRenderState, 0);
			}
		}

		Session* selectedSession = nullptr;
		std::unique_ptr<Document> addedDoc; int docReplacementIdx = -1;
		for (int sceneIdx = 0; sceneIdx < int(session && !session->scenes.empty() ? session->scenes.size() : 1) && show_ui; ++sceneIdx) {
			Document* document = session && sceneIdx < (int) session->scenes.size() ? session->scenes[sceneIdx].get() : nullptr;
			
			if (sceneIdx) {
				char buf[128];
				sprintf(buf, "Scene (%d)", sceneIdx + 1);
				ImGui::Begin(buf);
			}
			else
				ImGui::Begin("Interactive Preview", 0, ImGuiWindowFlags_AlwaysAutoResize);

			// session selector
			if (!sceneIdx)
			{
				char const* currentDocument = session ? session->name() : "<none>";
				if (ImGui::BeginCombo("Document", currentDocument)) {
					for (auto& s : sessions) {
						if (ImGui::Selectable(s->name(), s.get() == session))
							selectedSession = s.get();
					}
					if (session) {
						if (ImGui::Selectable("<add to session>")) {
							Session::AutoPause pause(session);
							addedDoc = browseForScene(config);
						}
					}
					if (ImGui::Selectable("<new session>")) {
						Session::AutoPause pause(session);
						auto newDoc = browseForScene(config);
						if (newDoc)
							selectedSession = openSession(std::move(newDoc));
					}
					if (session && !session->scenes.empty()) {
						ImGui::Selectable("-- replace in session: --", ImGuiSelectableFlags_Disabled);
						int sceneIdx = 0;
						for (auto& s : session->scenes) {
							if (ImGui::Selectable(s->filePath.s.c_str())) {
								Session::AutoPause pause(session);
								addedDoc = browseForScene(config);
								docReplacementIdx = sceneIdx;
							}
							++sceneIdx;
						}
					}
					ImGui::EndCombo();
				}
			}
			// reload?
			if (document && !addedDoc) {
				bool reload = ImGui::Button("reload");
				ImGui::SameLine();
				ImGui::Checkbox("track file changes", &track_file_changes);
				reload |= mouseSceneIdx == sceneIdx && !io.WantCaptureKeyboard && io.KeysDown[SDL_SCANCODE_F5] && !io.KeysDownDuration[SDL_SCANCODE_F5];
				reload |= track_file_changes && isTrackingFrame && document->fileChanged();
				if (reload) {
					addedDoc.reset( new Document(document->filePath, config) );
					docReplacementIdx = sceneIdx;
				}
			}

			if (document) {
				double spp = 0;
				for (int i = 0; i < document->renderer.integration.process->numActiveThreads; ++i) {
					spp += float(document->renderer.integration.samples[i]);
				}
				double sppPerS = spp / document->renderer.integration.timeSeconds();

				ImGui::Text("%dx%d @ %.1f spp (%.2f spp/s)", document->renderer.integration.preview->resX, document->renderer.integration.preview->resY, spp, sppPerS);
				if (mitsuba::ResponsiveIntegrator* igr = document->renderer.integration.process->integrator) {
					if (char const* stats = igr->getRealtimeStatistics())
						ImGui::Text("Stats: %s", stats);
				}
				if (ImGui::Button("Print Internal Stats"))
					Scene::printStats();
			}	
			if (!sceneIdx) {
				ImGui::Text("UI @ %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			}
			ImGui::NewLine();
			if (document) {
				ImGui::SliderInt("Refresh Rate", &document->renderer.integration.preview->readyMS, 0, 256);
				ImGui::SliderInt("Update Rate", &document->renderer.integration.preview->updateMS, 0, 512);
			}

			if (session) {
				{
					bool paused = session->paused();
					if (ImGui::Checkbox("Pause", &paused)) {
						session->pause(paused);
						if (!paused)
							show_final_render = false;
					}
					ImGui::SameLine();
					if (ImGui::Button("Restart"))
						session->restart();
					ImGui::SameLine();
					if (document)
						ImGui::Checkbox("Skip init", &document->renderer.skipInit);
					ImGui::NewLine();
				}
				if (!sceneIdx)
				{
					if (ImGui::Button("Render")) {
						session->startClassic();
						show_final_render = true;
					}
					ImGui::SameLine();
					if (ImGui::Button("Stop")) {
						session->stopClassic();
					}
					ImGui::SameLine();
					ImGui::Checkbox("Show Final", &show_final_render);
					ImGui::NewLine();
				}
			}

			if (document) {
				ImGui::SliderFloat("Cam Speed", &document->camera.speed, 0.01f, 100.f, "%.2f", 3);
				ImGui::SliderFloat("Cam Sensitivity", &document->camera.sensitivity, 0.01f, 10.f, "%.2f", 3);
				ImGui::Checkbox("Sync Cams", &sync_cams);
				if (ImGui::Button("Change Configuration"))
					document->startConfigurator();
			}

			if (!sceneIdx) {
				ImGui::SliderFloat("Exposure", &exposure, 0, 20, "%.2f", 4);
				ImGui::SameLine();
				ImGui::Checkbox("Flipping", &diff_flip);
				ImGui::Checkbox("Diff", &show_diff);
				ImGui::SameLine();
				ImGui::SliderFloat("Factor", &comparison_factor, 0, 10, "%.2f", 4);
				ImGui::SliderInt("Subres", &subres_levels, 0, 5);
				ImGui::SameLine();
				ImGui::Checkbox("Alpha", &alpha_transparent);
				ImGui::ColorEdit3("background", (float*)&clear_color);
			}
			ImGui::End();

			if (document && document->configurator) {
				bool config_open = true;
				if (sceneIdx) {
					char buf[128];
					sprintf(buf, "Configuration (%d)", sceneIdx + 1);
					ImGui::Begin(buf, &config_open);
				}
				else
					ImGui::Begin("Configuration", &config_open);

				if (document->configurator->run()) {
					if (!document->renderer.pendingChanges) {
						document->renderer.pendingChanges.reset( document->configurator->changes() );
						session->restart();
					}
				}

				ImGui::End();
				// user closed window
				if (!config_open)
					document->configurator.reset();
			}
		}
		if (!io.WantCaptureKeyboard && io.KeysDown[SDL_SCANCODE_PERIOD] && !io.KeysDownDuration[SDL_SCANCODE_PERIOD])
			show_ui = !show_ui;
		if (!io.WantCaptureKeyboard && io.KeysDown[SDL_SCANCODE_COMMA] && !io.KeysDownDuration[SDL_SCANCODE_COMMA])
			show_diff = !show_diff;
		if (!io.WantCaptureKeyboard && io.KeysDown[SDL_SCANCODE_BACKSLASH] && !io.KeysDownDuration[SDL_SCANCODE_BACKSLASH])
			diff_flip = !diff_flip;
		if (diff_flip || !io.WantCaptureKeyboard && (io.KeysDown[SDL_SCANCODE_SLASH] && !io.KeysDownDuration[SDL_SCANCODE_SLASH] || io.KeysDown[SDL_SCANCODE_SEMICOLON]))
			++scene_rotation_offset;

		// Rendering
		if (!null_render) {
			ImGui::Render();
			glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
			glScissor(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
			glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
			glClear(GL_COLOR_BUFFER_BIT);

			ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
			SDL_GL_SwapWindow(window);
		}
		else
			ImGui::EndFrame();
		
		if (session && mouseSceneIdx >= 0 && mouseSceneIdx < (int) session->scenes.size()) {
			auto& s = session->scenes[mouseSceneIdx];
			// todo: the ready check should not indefinitely delay changes ...
			bool changes = s->camera.update(io) && s->renderer.integration.preview->ready(currentTicks);
			if (changes) {
				if (sync_cams) {
					for (auto& s : session->scenes) {
						if (s != session->scenes[mouseSceneIdx])
							s->camera.transform(session->scenes[mouseSceneIdx]->camera.trafo);
					}
					session->restart();
				}
				else
					s->restart();
			}
		}

		if (addedDoc) {
			if (docReplacementIdx >= 0) {
				auto oldDoc = std::move(session->scenes[docReplacementIdx]);
				auto newDoc = addedDoc.get();
				session->scenes[docReplacementIdx] = std::move(addedDoc);
				(InteractiveTransform&) newDoc->camera = oldDoc->camera;
			}
			else
				session->scenes.push_back( std::move(addedDoc) );
			session->reconfigure(config);
			session->restart(); // restart old ones
			session->run(); // run new ones
		}
		
		if (selectedSession) {
			if (session)
				session->autoPause(true);
			session = selectedSession;
			selectedSession = nullptr;
			session->run();
		}
	}
}

int main(int argc, char** argv) {
	// Setup SDL
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_EVENTS) != 0)
	{
		printf("Error: %s\n", SDL_GetError());
		return -1;
	}

	mitsuba_start(argc, argv);

	// Setup window
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
//	SDL_GL_SetAttribute(SDL_GL_FRAMEBUFFER_SRGB_CAPABLE, 1); // breaks X forwarding, on by default anyways
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);
	SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 0);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
	SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	SDL_Window* window = SDL_CreateWindow("im-mitsuba", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
	SDL_GLContext gl_context = SDL_GL_CreateContext(window);
	SDL_GL_SetSwapInterval(1); // Enable vsync

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGuiContext* ui_context = ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsLight();

	// Setup Platform/Renderer bindings
	ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
	ImGui_ImplOpenGL2_Init();
	ImGui_ImplOpenGL2_NewFrame(); // init fonts

#if 0
	// Warmup
	Scene::availablePlugins("mitsuba_integrator_plugin", false);
	Scene::availablePlugins("mitsuba_film_plugin", false);
	Scene::availablePlugins("mitsuba_sensor_plugin", false);
#endif

	// Run
	run(argc, argv, window, gl_context, ui_context);

	// Cleanup
	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	SDL_GL_DeleteContext(gl_context);
	SDL_DestroyWindow(window);

	mitsuba_shutdown();

	SDL_Quit();

	return 0;
}