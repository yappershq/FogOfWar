using Sharp.Modules.AdminManager.Shared;
using Sharp.Shared;
using Sharp.Shared.Managers;

namespace FogOfWar;

/// <summary>
///     Holds the engine managers the Fog-of-War denial module consumes. The optional cross-plugin
///     <see cref="IAdminManager" /> (used only to gate the <c>fow_test</c> debug command) is resolved in
///     <c>OnAllModulesLoaded</c> — ModSharp guarantees all publishers' PostInit finished by then.
/// </summary>
internal sealed class InterfaceBridge
{
    internal string SharpPath { get; }

    internal IModSharp           ModSharp           { get; }
    internal ISharpModuleManager SharpModuleManager { get; }

    // Fog-of-War denial path: engine transmit hooks + entity/handle resolution + client listener. Line-of-sight
    // is computed off-thread against a BVH baked from the map's world_physics — no engine trace.
    internal ITransmitManager TransmitManager { get; }
    internal IEntityManager   EntityManager   { get; }
    internal IClientManager   ClientManager   { get; }

    // Smoke / HE occlusion: EventManager delivers hegrenade_detonate; SchemaManager resolves the smoke projectile's
    // m_bDidSmokeEffect field offset (the raw density-grid reads use fixed config offsets, not schema).
    internal IEventManager  EventManager  { get; }
    internal ISchemaManager SchemaManager { get; }

    // Resolved in OnAllModulesLoaded (optional — only the fow_test admin gate uses it).
    internal IAdminManager? AdminManager { get; private set; }

    public InterfaceBridge(string sharpPath, ISharedSystem sharedSystem)
    {
        SharpPath          = sharpPath;
        ModSharp           = sharedSystem.GetModSharp();
        SharpModuleManager = sharedSystem.GetSharpModuleManager();

        TransmitManager = sharedSystem.GetTransmitManager();
        EntityManager   = sharedSystem.GetEntityManager();
        ClientManager   = sharedSystem.GetClientManager();
        EventManager    = sharedSystem.GetEventManager();
        SchemaManager   = sharedSystem.GetSchemaManager();
    }

    internal void ResolveModules()
    {
        AdminManager = SharpModuleManager
            .GetOptionalSharpModuleInterface<IAdminManager>(IAdminManager.Identity)?.Instance;
    }
}
