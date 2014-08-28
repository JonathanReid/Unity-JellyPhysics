using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class CreatePressureBlock : MonoBehaviour {

    public GameObject baseObj;
    public GUIText text;
    private int _count=2;
    private List<GameObject> _gos = new List<GameObject>();
	
	// Update is called once per frame
	void Update () {
        if (Input.GetKeyDown(KeyCode.Joystick1Button0) || Input.GetKeyDown(KeyCode.Space))
        {
            GameObject go = Instantiate(baseObj,new Vector3(0,6,0),Quaternion.identity) as GameObject;
            _gos.Add(go);
            _count ++;
            text.text = _count.ToString();
        }

	}
}
