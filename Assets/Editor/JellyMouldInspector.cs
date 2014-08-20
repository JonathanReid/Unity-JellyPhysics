using UnityEditor;
using UnityEngine;
using System.Collections;

namespace JelloPhysics
{
	[CustomEditor (typeof(JellyMould))]
	public class JellyMouldInspector : UnityEditor.Editor {

		public override void OnInspectorGUI()
		{
			if (GUILayout.Button ("Open Editor")) 
			{
				JellyMouldEditor window = (JellyMouldEditor)EditorWindow.GetWindow (typeof (JellyMouldEditor));
				window.SetupWindow(target as JellyMould);
			}
		}
	}
}