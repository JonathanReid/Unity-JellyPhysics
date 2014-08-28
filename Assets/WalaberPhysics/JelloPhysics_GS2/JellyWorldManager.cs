using UnityEngine;
using System.Collections;
using System.Threading;

namespace JelloPhysics
{
	public class JellyWorldManager : MonoBehaviour {
	
		private static World _world;
		public static World World
		{
			get
			{
				return _world;
			}
		}

		private static JellyWorldManager _instance;
		public static JellyWorldManager Instance
		{
			get
			{
				if(_instance)
				{
					_instance = GameObject.FindObjectOfType<JellyWorldManager>();
				}
				if(_instance)
				{
					GameObject go = new GameObject();
					go.name = "JellyWorldManager";
					_instance = go.AddComponent<JellyWorldManager>();
				}
				return _instance;
			}
		}

        public static Vector2 Gravity = new Vector2(0,-9.8f);
        private Vector3 _cursorPos = Vector3.zero;
        private Body _dragBody = null;
        private int _dragPoint = -1;
        private Thread _physicsThread;
        private bool _cancelFlag;

		// Use this for initialization
		void Awake () {
            Application.targetFrameRate = 60;
			_world = new JelloPhysics.World();
            CreateFloor();
            _physicsThread = new Thread(UpdatePhysics){Name = "PhysicsThread"};
            _physicsThread.Start();
		}

        private void UpdatePhysics()
        {
            while (_cancelFlag == false)
            {
                for (int i = 0; i < 3; i++)
                    _world.update(1f/120f);

                Thread.Sleep(25);
            }
        }

		private void CreateFloor()
		{
			// static ground object.
			GameObject ground = new GameObject ();
			ClosedShape groundShape = new ClosedShape ();
			groundShape.begin();
			groundShape.addVertex(new Vector2(-10f, -2f));
			groundShape.addVertex(new Vector2(-10f, 2f));
			groundShape.addVertex(new Vector2(10f, 2f));
			groundShape.addVertex(new Vector2(10f, -2f));
			groundShape.finish();
			
			// make the body.
            Body groundBody = new Body();
			groundBody.Setup(_world, groundShape, float.PositiveInfinity, new Vector2(0f, -5f), 0f, Vector2.one, false);
		}

        void Update()
        {
            // UPDATE the physics!


            _cursorPos = Camera.main.ScreenToWorldPoint(Input.mousePosition);

            // dragging!
            if (Input.GetMouseButton(0))
            {
                if (_dragBody != null)
                {
                    PointMass pm = _dragBody.getPointMass(_dragPoint);
                    if (_dragBody.GetType().Name == "DraggableSpringBody")
                        ((DraggableSpringBody)_dragBody).setDragForce(JelloPhysics.VectorTools.calculateSpringForce(pm.Position, pm.Velocity, _cursorPos, Vector2.zero, 0.0f, 100.0f, 10.0f), _dragPoint);
                    else if (_dragBody.GetType().Name == "DraggablePressureBody")
                        ((DraggablePressureBody)_dragBody).setDragForce(JelloPhysics.VectorTools.calculateSpringForce(pm.Position, pm.Velocity, _cursorPos, Vector2.zero, 0.0f, 100.0f, 10.0f), _dragPoint);
                    
                }
            }
            else
            {
                _dragBody = null;
                _dragPoint = -1;
            }
            
            if (Input.GetMouseButtonDown(0))
            {
                if (_dragBody == (null))
                {
                    int body;
                    _world.getClosestPointMass(_cursorPos, out body, out _dragPoint);
                    _dragBody = _world.getBody(body);
                }
            }
        }
	}
}