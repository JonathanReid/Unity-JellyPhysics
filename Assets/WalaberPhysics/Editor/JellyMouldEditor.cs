using UnityEditor;
using UnityEngine;
using System.Collections;

namespace JelloPhysics
{
    public class JellyMouldEditor : EditorWindow
    {

        private bool _render;
        private JellyObject _target;
        private Texture _texture;
        private Vector2[] _points;
        private Vector2 _scrollPos;

        public void SetupWindow(JellyObject targetJelly)
        {
            _target = targetJelly;
            _texture = _target.GetComponent<SpriteRenderer>().sprite.texture;
            if (_target.GetComponent<PolygonCollider2D>())
            {
                _points = _target.GetComponent<PolygonCollider2D>().points;
            } 
            else
            {
                _points = _target.gameObject.AddComponent<PolygonCollider2D>().points;
            }
            _render = true;
        }

        void OnGUI()
        {
            if (!_render)
                return;
//            GUILayout.Label(_target.gameObject.name);

            GUILayout.BeginHorizontal();
            _scrollPos = GUILayout.BeginScrollView(_scrollPos);
//            GUILayout.Label(_texture);
            GUI.DrawTexture(new Rect(0, 0, _texture.width, _texture.height),_texture);
            if (_points != null && _points.Length > 0)
            {
//                Handles.DrawLine(_points [0], _points [1]);
                Handles.color = Color.red;
                Vector3 p1, p2;
                p1 = new Vector3(_texture.width, _texture.height, 0);

                int i = 1, l = _points.Length;
                for(;i<l;++i)
                {
                    p1 = Camera.main.WorldToScreenPoint(_points[i-1]*2);
                    p1.x -= _texture.width/2;
                    p1.z = 0;
                    p1.y = -p1.y + _texture.height;
//                    p1.x -= _texture.width/1.9f;
                    Debug.Log(p1);

                    p2 = Camera.main.WorldToScreenPoint(_points[i]*2);
                    p2.x -= _texture.width/2;
                    p2.z = 0;
                    p2.y = -p2.y + _texture.height;
//                    p2.x -= _texture.width/1.9f;
                    Handles.DrawLine(p1,p2);
                }
            }
            GUILayout.EndScrollView();
            GUILayout.EndHorizontal();
        }
    }   
}