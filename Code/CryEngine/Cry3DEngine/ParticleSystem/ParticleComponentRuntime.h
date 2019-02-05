// Copyright 2001-2018 Crytek GmbH / Crytek Group. All rights reserved.

#pragma once

#include "ParticleCommon.h"
#include "ParticleComponent.h"
#include "ParticleContainer.h"
#include <CryRenderer/IGpuParticles.h>

namespace pfx2
{

struct SInstance
{
	SInstance(TParticleId id = 0, float delay = 0.0f)
		: m_parentId(id), m_startDelay(delay) {}

	TParticleId m_parentId;
	float m_startDelay;
};

struct SMaxParticleCounts
{
	uint32 burst = 0;
	uint32 perFrame = 0;
	float  rate = 0;
};

class CParticleComponentRuntime : public _i_reference_target_t, public IParticleVertexCreator
{
public:
	CParticleComponentRuntime(CParticleEmitter* pEmitter, CParticleComponent* pComponent);
	~CParticleComponentRuntime();

	bool                          IsCPURuntime() const   { return !m_pGpuRuntime; }
	gpu_pfx2::IParticleComponentRuntime* GetGpuRuntime() const { return m_pGpuRuntime; }
	CParticleComponent*           GetComponent() const   { return m_pComponent; }
	bool                          IsValidForComponent() const;
	const AABB&                   GetBounds() const      { return m_pGpuRuntime ? m_pGpuRuntime->GetBounds() : m_bounds; }
	uint                          GetNumParticles() const;
	void                          AddBounds(const AABB& bounds);
	bool                          IsChild() const        { return m_pComponent->GetParentComponent() != nullptr; }
	void                          ReparentParticles(TConstArray<TParticleId> swapIds);
	void                          AddInstances(TVarArray<SInstance> instances);
	void                          RemoveAllSubInstances();
	void                          RenderAll(const SRenderContext& renderContext);

	void                          ComputeVertices(const SCameraInfo& camInfo, CREParticle* pRE, uint64 uRenderFlags, float fMaxPixels) override;

	void                      Initialize();
	void                      Clear();
	CParticleEffect*          GetEffect() const          { return m_pComponent->GetEffect(); }
	CParticleEmitter*         GetEmitter() const         { return m_pEmitter; }

	CParticleComponentRuntime* ParentRuntime() const;
	CParticleContainer&       GetParentContainer();
	const CParticleContainer& GetParentContainer() const;
	CParticleContainer&       GetContainer()            { return m_container; }
	const CParticleContainer& GetContainer() const      { return m_container; }

	void                      UpdateAll();
	void                      AddParticles(TConstArray<SSpawnEntry> spawnEntries);

	bool                      IsAlive() const               { return m_alive; }
	void                      SetAlive()                    { m_alive = true; }
	uint                      GetNumInstances() const       { return m_subInstances.size(); }
	uint                      GetDomainSize(EDataDomain domain) const;
	const SInstance&          GetInstance(uint idx) const   { return m_subInstances[idx]; }
	SInstance&                GetInstance(uint idx)         { return m_subInstances[idx]; }
	TParticleId               GetParentId(uint idx) const   { return GetInstance(idx).m_parentId; }
	template<typename T> T&   GetInstanceData(uint idx, TDataOffset<T> offset)
	{
		const auto stride = ComponentParams().m_instanceDataStride;
		CRY_PFX2_ASSERT(offset + sizeof(T) <= stride);
		CRY_PFX2_ASSERT(idx < m_subInstances.size());
		byte* addr = m_subInstanceData.data() + stride * idx + offset;
		return *reinterpret_cast<T*>(addr);
	}

	void                      GetMaxParticleCounts(int& total, int& perFrame, float minFPS = 4.0f, float maxFPS = 120.0f) const;
	void                      GetEmitLocations(TVarArray<QuatTS> locations, uint firstInstance) const;
	void                      EmitParticle();

	bool                      HasParticles() const;
	void                      AccumStats();

	SChaosKey&                Chaos() const           { return m_chaos; }
	SChaosKeyV&               ChaosV() const          { return m_chaosV; }

	SUpdateRange              FullRange() const       { return m_container.GetFullRange(); }
	SGroupRange               FullRangeV() const      { return SGroupRange(m_container.GetFullRange()); }
	SUpdateRange              SpawnedRange() const    { return m_container.GetSpawnedRange(); }
	SGroupRange               SpawnedRangeV() const   { return SGroupRange(m_container.GetSpawnedRange()); }
	const SComponentParams&   ComponentParams() const { return m_pComponent->GetComponentParams(); }

	static TParticleHeap&     MemHeap();
	float                     DeltaTime() const;
	bool                      IsPreRunning() const    { return m_isPreRunning; }

private:
	void PreRun();
	void AddInstances();
	void AddRemoveParticles();
	void RemoveParticles();
	void InitParticles();
	void AgeUpdate();
	void UpdateParticles();
	void CalculateBounds();
	void DebugStabilityCheck();
	void UpdateGPURuntime();

	_smart_ptr<CParticleComponent> m_pComponent;
	CParticleEmitter*              m_pEmitter;
	CParticleContainer             m_container;
	TDynArray<SInstance>           m_subInstances;
	TDynArray<byte>                m_subInstanceData;
	AABB                           m_bounds;
	bool                           m_alive;
	bool                           m_isPreRunning;
	float                          m_deltaTime;
	SChaosKey mutable              m_chaos;
	SChaosKeyV mutable             m_chaosV;

	_smart_ptr<gpu_pfx2::IParticleComponentRuntime> m_pGpuRuntime;
};

template<typename T>
struct SDynamicData : THeapArray<T>
{
	SDynamicData(const CParticleComponentRuntime& runtime, EParticleDataType type, EDataDomain domain, SUpdateRange range)
		: THeapArray<T>(runtime.MemHeap(), range.size())
	{
		memset(this->data(), 0, this->size_mem());
		runtime.GetComponent()->GetDynamicData(runtime, type, this->data(), domain, range);
	}
};



}

