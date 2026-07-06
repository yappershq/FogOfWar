using System;
using System.IO;
using System.Numerics;
using System.Runtime.InteropServices;

namespace FogOfWar.Engine;

/// <summary>
///     Serialises / deserialises a built <see cref="LosQuery" /> (triangle soup + BVH) to a binary file keyed
///     by the map's <c>world_physics</c> CRC32 + uncompressed size, mirroring upstream CS2FOW's CRC-keyed
///     <c>.bvh8</c> cache. On map load the caller checks <see cref="TryLoad" /> against the current map's
///     CRC/size; on a miss it builds via <see cref="LosQuery.Build" /> and calls <see cref="Save" />.
///
///     <para>The format is a fixed header followed by four raw, blittable blocks (v0/v1/v2 as
///     <see cref="Vector3" />, the triangle-index permutation as <c>int</c>, and the <see cref="BvhNode" />
///     pool). Load is a single <c>File.ReadAllBytes</c> + <c>MemoryMarshal.Cast</c> — no per-element parsing —
///     so it is dramatically faster than re-running VRF + the BVH build.</para>
/// </summary>
internal static class BakeCache
{
    private const uint Magic   = 0x57_4F_46_42; // "FOWB" (little-endian: 'F','O','W','B')
    // v2: the opacity filter changed which triangles are occluders (interaction-layer + thin-geometry gates),
    // so v1 caches hold stale occluder sets — a bump forces every map to re-bake with the current filter
    // (TryLoad rejects the old version; Save overwrites in place since the filename is CRC/size-keyed).
    private const int  Version = 2;

    // Header: magic, version, crc32, size(long), triCount, nodeCount.
    private const int HeaderBytes = 4 + 4 + 4 + 8 + 4 + 4;

    public static string FileName(uint crc32, long size) => $"fow_{crc32:x8}_{size}.bvh";

    /// <summary>Full cache path for a map's physics CRC/size under <paramref name="dir" />.</summary>
    public static string PathFor(string dir, uint crc32, long size)
        => Path.Combine(dir, FileName(crc32, size));

    /// <summary>
    ///     Try to load a cached query whose header matches <paramref name="crc32" /> + <paramref name="size" />.
    ///     Returns false (and null) on a missing file, a header mismatch, or any corruption — the caller then
    ///     rebuilds. Never throws.
    /// </summary>
    public static bool TryLoad(string dir, uint crc32, long size, out LosQuery? query)
    {
        query = null;
        var path = PathFor(dir, crc32, size);

        try
        {
            if (!File.Exists(path))
                return false;

            var bytes = File.ReadAllBytes(path);
            if (bytes.Length < HeaderBytes)
                return false;

            var span = bytes.AsSpan();
            var off = 0;
            if (ReadUInt(span, ref off) != Magic) return false;
            if (ReadInt(span, ref off)  != Version) return false;
            var fileCrc = ReadUInt(span, ref off);
            var fileSize = ReadLong(span, ref off);
            var triCount = ReadInt(span, ref off);
            var nodeCount = ReadInt(span, ref off);

            if (fileCrc != crc32 || fileSize != size || triCount < 0 || nodeCount < 1)
                return false;

            long need = (long)HeaderBytes
                        + 3L * triCount * Marshal.SizeOf<Vector3>()
                        + (long)triCount * sizeof(int)
                        + (long)nodeCount * Marshal.SizeOf<BvhNode>();
            if (bytes.Length != need)
                return false;

            var v0 = new Vector3[triCount];
            var v1 = new Vector3[triCount];
            var v2 = new Vector3[triCount];
            var triIdx = new int[triCount];
            var nodes = new BvhNode[nodeCount];

            ReadBlock(span, ref off, MemoryMarshal.AsBytes(v0.AsSpan()));
            ReadBlock(span, ref off, MemoryMarshal.AsBytes(v1.AsSpan()));
            ReadBlock(span, ref off, MemoryMarshal.AsBytes(v2.AsSpan()));
            ReadBlock(span, ref off, MemoryMarshal.AsBytes(triIdx.AsSpan()));
            ReadBlock(span, ref off, MemoryMarshal.AsBytes(nodes.AsSpan()));

            var bvh = new Bvh(v0, v1, v2, nodes, triIdx, nodeCount);

            // The header CRC only covers the source world_physics data, NOT this serialised BVH blob — a
            // truncated / bit-rotted / hand-tampered cache can pass the size check yet carry a corrupt node
            // graph whose child indices cycle, hanging the worker's traversal forever. Structurally validate
            // before trusting it; on any violation reject → the caller rebuilds from VRF.
            if (!bvh.Validate())
                return false;

            query = new LosQuery(bvh, crc32, size);
            return true;
        }
        catch
        {
            query = null;
            return false;
        }
    }

    /// <summary>
    ///     Serialise a built query to its CRC/size-keyed file under <paramref name="dir" /> (created if
    ///     absent). Writes to a temp file then atomically moves into place. Never throws — a cache-write
    ///     failure only costs a rebuild next map load.
    /// </summary>
    public static bool Save(string dir, LosQuery query)
    {
        try
        {
            Directory.CreateDirectory(dir);

            var bvh = query.Bvh;
            var tri = bvh.TriangleCount;
            var nodeCount = bvh.NodeCount;

            long total = (long)HeaderBytes
                         + 3L * tri * Marshal.SizeOf<Vector3>()
                         + (long)tri * sizeof(int)
                         + (long)nodeCount * Marshal.SizeOf<BvhNode>();

            var path = PathFor(dir, query.Crc32, query.Size);
            var tmp = path + ".tmp";

            using (var fs = new FileStream(tmp, FileMode.Create, FileAccess.Write, FileShare.None,
                       1 << 20, FileOptions.SequentialScan))
            {
                Span<byte> header = stackalloc byte[HeaderBytes];
                var off = 0;
                WriteUInt(header, ref off, Magic);
                WriteInt(header, ref off, Version);
                WriteUInt(header, ref off, query.Crc32);
                WriteLong(header, ref off, query.Size);
                WriteInt(header, ref off, tri);
                WriteInt(header, ref off, nodeCount);
                fs.Write(header);

                fs.Write(MemoryMarshal.AsBytes(bvh.V0.AsSpan(0, tri)));
                fs.Write(MemoryMarshal.AsBytes(bvh.V1.AsSpan(0, tri)));
                fs.Write(MemoryMarshal.AsBytes(bvh.V2.AsSpan(0, tri)));
                fs.Write(MemoryMarshal.AsBytes(bvh.TriIdx.AsSpan(0, tri)));
                fs.Write(MemoryMarshal.AsBytes(bvh.Nodes.AsSpan(0, nodeCount)));
            }

            if (File.Exists(path))
                File.Delete(path);
            File.Move(tmp, path);
            return true;
        }
        catch
        {
            return false;
        }
    }

    // ── little-endian primitive read/write ───────────────────────────────────────────────────────────────

    private static void ReadBlock(ReadOnlySpan<byte> src, ref int off, Span<byte> dst)
    {
        src.Slice(off, dst.Length).CopyTo(dst);
        off += dst.Length;
    }

    private static uint ReadUInt(ReadOnlySpan<byte> s, ref int o)
    { var v = BitConverter.ToUInt32(s.Slice(o, 4)); o += 4; return v; }

    private static int ReadInt(ReadOnlySpan<byte> s, ref int o)
    { var v = BitConverter.ToInt32(s.Slice(o, 4)); o += 4; return v; }

    private static long ReadLong(ReadOnlySpan<byte> s, ref int o)
    { var v = BitConverter.ToInt64(s.Slice(o, 8)); o += 8; return v; }

    private static void WriteUInt(Span<byte> s, ref int o, uint v)
    { BitConverter.TryWriteBytes(s.Slice(o, 4), v); o += 4; }

    private static void WriteInt(Span<byte> s, ref int o, int v)
    { BitConverter.TryWriteBytes(s.Slice(o, 4), v); o += 4; }

    private static void WriteLong(Span<byte> s, ref int o, long v)
    { BitConverter.TryWriteBytes(s.Slice(o, 8), v); o += 8; }
}
