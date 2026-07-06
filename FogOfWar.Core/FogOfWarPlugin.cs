using FogOfWar.Configuration;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging;
using Sharp.Shared;

namespace FogOfWar;

/// <summary>
///     FogOfWar — standalone ModSharp anti-wallhack DENIAL plugin. Concept ported from CS2FOW by karola3vax
///     (MIT). NOT a detector: it never warns / kicks / bans. Instead it stops the server from networking an
///     enemy pawn to a client that provably cannot see it, so a wallhack / ESP has no pawn state (position /
///     animation / health pose) to read.
///
///     <para>Visibility is computed off the game thread against a BVH baked from the map's static
///     <c>world_physics</c> collision (opacity-filtered so fences / grates / glass do not occlude), under a
///     strict fail-open rule — any degradation errs VISIBLE, never hidden.</para>
///
///     <para>Lifecycle: the bridge + config are built in the constructor; the optional cross-plugin
///     <c>IAdminManager</c> (used only for the <c>fow_test</c> debug gate) is resolved in
///     <c>OnAllModulesLoaded</c> (ModSharp guarantees all publishers' PostInit finished by then), and the
///     module — which installs the transmit hooks, the game/client/entity listeners and starts its background
///     worker — is installed there. Opt-in: inert unless <c>fogOfWar.enabled</c> is set in
///     <c>configs/fogofwar.json</c>.</para>
/// </summary>
public sealed class FogOfWarPlugin : IModSharpModule
{
    public string DisplayName   => "FogOfWar";
    public string DisplayAuthor => "yappershq (concept from CS2FOW by karola3vax, MIT)";

    private readonly ILogger<FogOfWarPlugin> _logger;
    private readonly InterfaceBridge         _bridge;
    private readonly FogOfWarConfig          _config;
    private readonly FogOfWarModule          _fogOfWar;

    public FogOfWarPlugin(
        ISharedSystem  sharedSystem,
        string         dllPath,
        string         sharpPath,
        System.Version version,
        IConfiguration configuration,
        bool           hotReload)
    {
        var loggerFactory = sharedSystem.GetLoggerFactory();
        _logger  = loggerFactory.CreateLogger<FogOfWarPlugin>();

        _bridge   = new InterfaceBridge(sharpPath, sharedSystem);
        _config   = FogOfWarConfig.Load(sharpPath, loggerFactory.CreateLogger<FogOfWarConfig>());
        _fogOfWar = new FogOfWarModule(_bridge, _config, loggerFactory.CreateLogger<FogOfWarModule>());
    }

    public bool Init() => true;

    public void PostInit() { }

    public void OnAllModulesLoaded()
    {
        // Resolve the optional AdminManager (fow_test debug gate) before installing — ModSharp guarantees all
        // publishers' PostInit finished by now.
        _bridge.ResolveModules();

        // DENIAL — transmit-culls enemies a client can't see. Opt-in via fogOfWar.enabled (inert by default);
        // Install installs the hooks + game/client/entity listeners and starts the background worker.
        _fogOfWar.Install();

        _logger.LogInformation(
            "[FogOfWar] Loaded — fogOfWar={Fow}, AdminManager={Mgr}",
            _fogOfWar.IsInstalled, _bridge.AdminManager is not null);
    }

    public void OnLibraryConnected(string name)
    {
        // Re-resolve the optional AdminManager if it (re)connects after our OAM, so a hot-reload of that module
        // doesn't leave us holding a stale/null reference.
        if (name is "AdminManager")
            _bridge.ResolveModules();
    }

    public void OnLibraryDisconnect(string name) { }

    public void Shutdown()
    {
        _fogOfWar.Uninstall();
    }
}
