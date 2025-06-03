using UnityEditor;
using UnityEngine;

public class AutoImportProcessor : AssetPostprocessor
{
    void OnPreprocessTexture()
    {
        if (assetPath.StartsWith("Assets/Maps/") && assetPath.EndsWith(".png"))
        {
            var ti = (TextureImporter)assetImporter;
            ti.textureType    = TextureImporterType.Default;
            ti.wrapMode       = TextureWrapMode.Clamp;
            ti.filterMode     = FilterMode.Point;
            ti.maxTextureSize = 4096;
            ti.isReadable     = true;
            ti.SaveAndReimport();
        }
    }
}
