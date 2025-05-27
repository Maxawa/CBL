using System.Collections;
using System.Collections.Generic;
using System.Data.Common;
using System.IO;
using Unity.Collections;
using Unity.Robotics.UrdfImporter;
using Unity.VisualScripting;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

public class MapReader : MonoBehaviour
{
    private string[] data;
    public Texture2D tex;
    // Start is called before the first frame update
    void Start()
    {
        using (StreamReader reader = new StreamReader(Path.Combine(Application.streamingAssetsPath, "map.pgm")))
        {
            int lineNumber = 0;
            string line;
            while ((line = reader.ReadLine()) != null)
            {
                if (line[0] != '#')
                { //exclude comments
                    lineNumber++;
                }
                if (lineNumber == 2)
                {
                    string[] size = line.Split(" ");
                    tex.Reinitialize(int.Parse(size[0]), int.Parse(size[1]), TextureFormat.RGB24, false); //create the texture
                }
                if (lineNumber == 4)
                {
                    for (int count = 0; count < line.Length; count++)
                    {
                    int x = count % tex.width;
                    int y = count / tex.width;
                    float greyCol = line[count];
                    if (greyCol < 50)
                    {
                        tex.SetPixel(x, y, Color.blue);
                    }
                    else if (greyCol < 210)
                    {
                        tex.SetPixel(x, y, Color.grey);
                    }
                    else
                    {
                        tex.SetPixel(x, y, Color.cyan);
                    }
                    }
                tex.Apply();
                }
            }
        }
    }

    // Update is called once per frame
    void Update() {
        
    }
}
