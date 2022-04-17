using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

/// <summary>
/// Cloth physics manager. Add this to a Scene Object with a mesh and flow the wind
/// </summary>


public class ClothBehaviour : MonoBehaviour
{
    #region OtherVariables

    [HideInInspector] Mesh m_Mesh;

    [HideInInspector] private Vector3[] m_Vertices;
    [HideInInspector] private int[] m_Triangles;

    [HideInInspector] private List<Node> m_Nodes;
    [HideInInspector] private List<Spring> m_Springs;

    [HideInInspector] private List<Node> m_FixedNodes;

    [HideInInspector] private WindZone[] m_WindObjs;

    [HideInInspector] private Vector3 m_AverageWindVelocity;

    #endregion 


    #region InEditorVariables

    [SerializeField] [Range(0.001f, 0.02f)] private float m_TimeStep;
    [SerializeField] [Range(1, 20)] private int m_Substeps;

    [SerializeField] public Vector3 m_Gravity;

    [SerializeField] private bool m_Paused;

    [SerializeField] private float m_TractionStiffness;
    [SerializeField] private float m_FlexionStiffness;

    [SerializeField] private float m_NodeMass;

    [SerializeField] private float m_NodeDamping;
    [SerializeField] private float m_SpringDamping;

    [SerializeField] private Solver m_SolvingMethod;

    #endregion

    #region ConditionalInEditorVariables
    //------Toggled by custom editor only if enabled-----//

    [HideInInspector] public bool m_AffectedByWind;
    [HideInInspector] [Range(0, 1)] public float m_WindFriction;

    [HideInInspector] public bool m_FixingByTexture;
    [HideInInspector] public Texture2D m_Texture;
    [HideInInspector] public List<GameObject> m_Fixers;

    [HideInInspector] public bool m_CanCollide;
    [HideInInspector] public List<GameObject> m_CollidingObjects;
    [HideInInspector] public PenaltyPrecission m_PenaltyForcePrecission;

    #endregion

    public ClothBehaviour()
    {
        m_FixingByTexture = false;

        m_TimeStep = 0.004f;
        m_Substeps = 5;

        m_Gravity = new Vector3(0.0f, -9.81f, 0.0f);

        m_Paused = true;

        m_TractionStiffness = 20f;
        m_FlexionStiffness = 15f;

        m_NodeMass = 0.03f;

        m_NodeDamping = 0.3f;
        m_SpringDamping = 0.3f;

        m_SolvingMethod = Solver.Symplectic;

        m_FixedNodes = new List<Node>();

        m_AverageWindVelocity = Vector3.zero;

        m_AffectedByWind = false;
        m_WindFriction = 0.5f;

        m_CanCollide = false;
        m_PenaltyForcePrecission = PenaltyPrecission.High;
    }

    public enum Solver
    {
        Explicit = 0,
        Symplectic = 1,
        Midpoint = 2,
    };
    public enum PenaltyPrecission
    {
        Low = 4,
        Medium = 2,
        High = 1,
    }


    #region MonoBehaviour

    public void Start()
    {
        m_Mesh = GetComponent<MeshFilter>().mesh;
        m_Vertices = m_Mesh.vertices;
        m_Triangles = m_Mesh.triangles;

        Vector2[] vertex_UVs = m_Mesh.uv;

        m_Nodes = new List<Node>();

        for (int i = 0; i < m_Mesh.vertexCount; i++)
        {
            //For simulation purposes, transform the points to global coordinates
            m_Nodes.Add(new Node(i, transform.TransformPoint(m_Vertices[i]), this, m_NodeDamping, m_NodeMass, vertex_UVs[i]));
        }


        m_Springs = new List<Spring>();

        EdgeQualityComparer edgeComparer = new EdgeQualityComparer();

        Dictionary<Edge, Edge> edgeDictionary = new Dictionary<Edge, Edge>(edgeComparer);

        for (int i = 0; i < m_Mesh.triangles.Length; i += 3)
        {
            List<Edge> edges = new List<Edge>();
            edges.Add(new Edge(m_Triangles[i], m_Triangles[i + 1], m_Triangles[i + 2]));
            edges.Add(new Edge(m_Triangles[i + 1], m_Triangles[i + 2], m_Triangles[i]));
            edges.Add(new Edge(m_Triangles[i], m_Triangles[i + 2], m_Triangles[i + 1]));
            foreach (var edge in edges)
            {
                Edge otherEdge;
                if (edgeDictionary.TryGetValue(edge, out otherEdge))
                {
                    //La arista está en el diccionario
                    otherEdge.m_A = otherEdge.m_O;
                    otherEdge.m_B = edge.m_O;
                    //edgeDictionary.Add(otherEdge, otherEdge);
                    m_Springs.Add(new Spring(m_Nodes[otherEdge.m_A], m_Nodes[otherEdge.m_B], this, m_FlexionStiffness, m_SpringDamping));

                }
                else
                {
                    //La arista no está en el diccionario
                    edgeDictionary.Add(edge, edge);
                    m_Springs.Add(new Spring(m_Nodes[edge.m_A], m_Nodes[edge.m_B], this, m_TractionStiffness, m_SpringDamping));
                }
            }
        }
        ////
        //string edg = "";
        //for (int i = 0; i < m_Springs.Count; i++)
        //{
        //    edg += m_Springs[i].m_NodeA.m_Id + "" + m_Springs[i].m_NodeB.m_Id + "   ";
        //}
        //Debug.Log(edg);
        ////

        //List<Edge> edges = new List<Edge>();

        //for (int i = 0; i < m_Mesh.triangles.Length; i += 3)
        //{
        //    edges.Add(new Edge(m_Triangles[i], m_Triangles[i + 1], m_Triangles[i + 2]));
        //    edges.Add(new Edge(m_Triangles[i + 1], m_Triangles[i + 2], m_Triangles[i]));
        //    edges.Add(new Edge(m_Triangles[i], m_Triangles[i + 2], m_Triangles[i + 1]));

        //}
        ////
        //string edg = "";
        //for (int i = 0; i < edges.Count; i++)
        //{
        //    edg += edges[i].m_A + "" + edges[i].m_B + "" + edges[i].m_O + "   ";
        //}
        //Debug.Log(edg);
        ////
        //EdgeComparer edgeComparer = new EdgeComparer();

        //edges.Sort(edgeComparer);
        ////
        //edg = "";
        //for (int i = 0; i < edges.Count; i++)
        //{
        //    edg += edges[i].m_A + "" + edges[i].m_B + "" + edges[i].m_O + "   ";
        //}
        //Debug.Log(edg);
        ////
        //m_Springs.Add(new Spring(m_Nodes[edges[0].m_A], m_Nodes[edges[0].m_B], this, m_TractionStiffness));

        //for (int i = 1; i < m_Mesh.triangles.Length; i++)
        //{
        //    if (edges[i].m_A == edges[i - 1].m_A && edges[i].m_B == edges[i - 1].m_B || edges[i].m_A == edges[i - 1].m_A && edges[i].m_B == edges[i - 1].m_B)
        //    {
        //        //Flexion
        //        edges[i].m_A = edges[i].m_O;
        //        edges[i].m_B = edges[i - 1].m_O;
        //        m_Springs.Add(new Spring(m_Nodes[edges[i].m_A], m_Nodes[edges[i].m_B], this, m_FlexionStiffness));

        //    }
        //    else
        //    {
        //        //Tracción
        //        m_Springs.Add(new Spring(m_Nodes[edges[i].m_A], m_Nodes[edges[i].m_B], this, m_TractionStiffness));
        //    }
        //}
        ////
        //edg = "";
        //for (int i = 0; i < edges.Count; i++)
        //{
        //    edg += edges[i].m_A + "" + edges[i].m_B + "" + edges[i].m_O + "   ";
        //}
        //Debug.Log(edg);
        ////

        m_TimeStep = m_TimeStep / m_Substeps;

        //Attach to fixers
        if (m_FixingByTexture)
            CheckTextureWeights();
        else
            CheckFixers();

        //Look for Wind objs
        CheckWindObjects();

    }



    public void Update()
    {
        if (Input.GetKeyUp(KeyCode.P))
            this.m_Paused = !this.m_Paused;


        CheckWindObjects();


        //ACTUALIZAR POSICION DE NODOS CUANDO SE MUEVEN
        foreach (var node in m_FixedNodes)
        {
            if (m_FixingByTexture)
            {
                node.m_Pos = transform.TransformPoint(node.m_offset);
            }
            else
            {
                node.m_Pos = node.m_Fixer.transform.TransformPoint(node.m_offset);

            }
        }


        for (int i = 0; i < m_Mesh.vertexCount; i++)
        {
            m_Vertices[i] = transform.InverseTransformPoint(m_Nodes[i].m_Pos);
        }

        m_Mesh.vertices = m_Vertices;
    }

    public void FixedUpdate()
    {
        if (this.m_Paused)
            return; // Not simulating

        if (m_AffectedByWind)
        {
            ComputeWindForces();
        }
        // Select integration method
        for (int i = 0; i < m_Substeps; i++)
        {

            switch (this.m_SolvingMethod)
            {

                case Solver.Explicit: this.stepExplicit(); break;

                case Solver.Symplectic: this.stepSymplectic(); break;

                case Solver.Midpoint: this.stepRK2(); break;

                default:
                    throw new System.Exception("[ERROR] Should never happen!");

            }
        }

    }

    #endregion
    #region PhysicsSolvers
    /// <summary>
    /// Worst solver. Good for simple assets or arcade physics
    /// </summary>
    private void stepExplicit()
    {
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;

            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                n.m_Pos += m_TimeStep * n.m_Vel;
                n.m_Vel += m_TimeStep / n.m_Mass * n.m_Force;
            }
        }

    }
    /// <summary>
    /// Better solver. Either not perfect
    /// </summary>
    private void stepSymplectic()
    {

        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;
            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                n.m_Vel += m_TimeStep / n.m_Mass * n.m_Force;
                n.m_Pos += m_TimeStep * n.m_Vel;
            }
        }

    }
    /// <summary>
    /// Fairly good solver
    /// </summary>
    private void stepRK2()
    {
        Vector3[] m_Vel0 = new Vector3[m_Nodes.Count];
        Vector3[] m_Pos0 = new Vector3[m_Nodes.Count];

        //Midpoint
        for (int i = 0; i < m_Nodes.Count; i++)
        {

            m_Vel0[i] = m_Nodes[i].m_Vel;
            m_Pos0[i] = m_Nodes[i].m_Pos;


            m_Nodes[i].m_Force = Vector3.zero;


            m_Nodes[i].ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                n.m_Vel += (m_TimeStep * 0.5f) / n.m_Mass * n.m_Force;
                n.m_Pos += (m_TimeStep * 0.5f) * n.m_Vel;
            }
        }

        //EndPoint
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;

            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        for (int i = 0; i < m_Nodes.Count; i++)
        {

            if (!m_Nodes[i].m_Fixed)
            {
                m_Nodes[i].m_Vel = m_Vel0[i] + m_TimeStep / m_Nodes[i].m_Mass * m_Nodes[i].m_Force;
                m_Nodes[i].m_Pos = m_Pos0[i] + m_TimeStep * m_Nodes[i].m_Vel;
            }
        }


    }
    #endregion
    /// <summary>
    /// Checks whether the vertex is inside the fixers colliders in order to put it in fixer state
    /// </summary>
    private void CheckFixers()
    {
        foreach (var node in m_Nodes)
        {
            foreach (var obj in m_Fixers)
            {
                Collider collider = obj.GetComponent<Collider>();
                Vector3 n_pos = node.m_Pos;

                if (collider.bounds.Contains(n_pos))
                {
                    node.m_Fixed = true;
                    if (node.m_Fixer != null) Debug.LogWarning("More than one fixer assinged to the vertex. It will only be accepted one");
                    node.m_Fixer = obj;
                    node.m_offset = node.m_Fixer.transform.InverseTransformPoint(node.m_Pos);
                    m_FixedNodes.Add(node);

                }

                //PROGRAMMED COLLISION ALGORITHMS

                //Vector3 f_center = collider.bounds.center;
                //    if (collider.GetType() == typeof(SphereCollider))
                //    {
                //        //Simple collision function with spheres
                //        float f_radius = obj.GetComponent<SphereCollider>().radius;
                //        float dist = (n_pos - f_center).magnitude;

                //        if (dist <= f_radius)
                //        {
                //            node.m_Fixed = true;
                //            if (node.m_Fixer != null) Debug.LogWarning("More than one fixer assinged to the vertex. It will only be accepted one");
                //            node.m_Fixer = obj;
                //            node.m_offset = node.m_Pos - node.m_Fixer.transform.position;
                //            m_FixedNodes.Add(node);
                //        }

                //    }
                //    else
                //    {
                //        //Simple collision function with boxes
                //        Vector3 f_size = collider.bounds.extents;
                //        //f_size = transform.InverseTransformPoint(f_size);

                //        float x_dist = Mathf.Abs(f_center.x - n_pos.x);
                //        float y_dist = Mathf.Abs(f_center.y - n_pos.y);
                //        float z_dist = Mathf.Abs(f_center.z - n_pos.z);


                //        int condition = 0;
                //        if (x_dist <= f_size.x) condition++;
                //        if (y_dist <= f_size.y) condition++;
                //        if (z_dist <= f_size.z) condition++;

                //        if (condition == 3)
                //        {
                //            node.m_Fixed = true;
                //            if (node.m_Fixer != null) Debug.LogWarning("More than one fixer assinged to the vertex. It will only be accepted one");
                //            node.m_Fixer = obj;
                //            node.m_offset = node.m_Pos - node.m_Fixer.transform.position;
                //            m_FixedNodes.Add(node);
                //        }

                //    }


            }

        }
    }
    private void CheckTextureWeights()
    {

        Vector2[] uvs = m_Mesh.uv;
        int textWidth = m_Texture.width;


        foreach (var node in m_Nodes)
        {
            float xCoord = textWidth * node.m_UV.x;
            float yCoord = textWidth * node.m_UV.y;

            Color color = m_Texture.GetPixel((int)xCoord, (int)yCoord);

            float factor = 0.9f;
            if (color.a >= factor)
            {
                node.m_Fixed = true;
                m_FixedNodes.Add(node);
                node.m_offset = transform.InverseTransformPoint(node.m_Pos);
            }
            else
            {
                node.m_ForceFactor = 1 - color.a;
            }
        }
        //foreach (var item in m_FixedNodes) 
        //{
        //    Debug.Log(item);
        //}
    }
    /// <summary>
    /// Automatically called on start. Checks for wind objects in order to take them into account to make the wind simulation
    /// </summary>
    private void CheckWindObjects()
    {
        if (m_AffectedByWind) m_WindObjs = FindObjectsOfType<WindZone>();
        ComputeWindSpeed();
    }
    /// <summary>
    /// Called every update. Computes the average velocity vector of the resulting wind.
    /// </summary>
    private void ComputeWindSpeed()
    {
        m_AverageWindVelocity = Vector3.zero;

        if (m_AffectedByWind)
        {
            int total = m_WindObjs.Length;
            foreach (var obj in m_WindObjs)
            {
                Vector3 windVel = obj.transform.forward * (obj.windMain + (obj.windPulseMagnitude * obj.windMain * Mathf.Abs(Mathf.Sin(Time.time * (obj.windPulseFrequency * 10))))); //tweaked
                m_AverageWindVelocity += windVel;

            }
            m_AverageWindVelocity = new Vector3(m_AverageWindVelocity.x / total, m_AverageWindVelocity.y / total, m_AverageWindVelocity.z / total);
        }


    }
    /// <summary>
    /// Called every step update in the fixed update. Computes the applied wind force of every triangle in the mesh. 
    /// </summary>
    private void ComputeWindForces()
    {

        for (int i = 0; i < m_Mesh.triangles.Length; i += 3)
        {

            Node nodeA = m_Nodes[m_Triangles[i]];
            Node nodeB = m_Nodes[m_Triangles[i + 1]];
            Node nodeC = m_Nodes[m_Triangles[i + 2]];

            //Area triangulo FORMULA DE HERON
            float sideA = (nodeA.m_Pos - nodeB.m_Pos).magnitude;
            float sideB = (nodeB.m_Pos - nodeC.m_Pos).magnitude;
            float sideC = (nodeC.m_Pos - nodeA.m_Pos).magnitude;
            float semiPerimeter = (sideA + sideB + sideC) / 2;

            float trisArea = Mathf.Sqrt(semiPerimeter * (semiPerimeter - sideA) * (semiPerimeter - sideB) * (semiPerimeter - sideC));

            Vector3 trisNormal = Vector3.Cross(nodeA.m_Pos - nodeB.m_Pos, nodeB.m_Pos - nodeC.m_Pos).normalized;


            //Velocidad triangulo MEDIA DE LAS VELOCIDADES DE LOS NODES
            Vector3 trisSpeed = new Vector3((nodeA.m_Vel.x + nodeB.m_Vel.x + nodeC.m_Vel.x) / 3, (nodeA.m_Vel.y + nodeB.m_Vel.y + nodeC.m_Vel.y) / 3, (nodeA.m_Vel.z + nodeB.m_Vel.z + nodeC.m_Vel.z) / 3);

            Vector3 trisWindForce = m_WindFriction * trisArea * Vector3.Dot(trisNormal, m_AverageWindVelocity - trisSpeed) * trisNormal;
            Vector3 dispersedForce = trisWindForce / 3;

            //Dotar a los nodos de la fuerza dispersada entre 3
            nodeA.m_WindForce = dispersedForce;
            nodeB.m_WindForce = dispersedForce;
            nodeC.m_WindForce = dispersedForce;
        }
    }


}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//AUXILIAR CLASSES

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/// <summary>
/// Class representing each vertex physical attributes necessary
/// to compute the cloth physics
/// </summary>
public class Node
{
    public bool m_Fixed;

    public readonly int m_Id;
    public Vector3 m_Pos;
    public Vector3 m_Vel;
    public Vector3 m_Force;
    public float m_ForceFactor;
    public float m_Mass;
    public float m_DampingA;

    public Vector3 m_offset;            //Only if fixed enabled

    public ClothBehaviour m_Manager;
    public GameObject m_Fixer;          //Only if fixed enabled

    public Vector3 m_WindForce;         //Only if wind enabled
    public Vector3 m_PenaltyForce;      //Only if collisions enabled

    public Vector2 m_UV;


    public Node(int iD, Vector3 pos, ClothBehaviour manager, float dampA, float mass, Vector2 uv)
    {
        m_Fixed = false;

        m_Id = iD;
        m_Pos = pos;
        m_DampingA = dampA;
        m_Vel = Vector3.zero;
        m_Manager = manager;
        m_Mass = mass;

        m_Fixer = null;

        m_UV = uv;

        m_ForceFactor = 1f;

        //m_offset = manager.transform.InverseTransformPoint(m_Pos);

    }
    public void UpdateVariables(float dampA, float dampB, float mass)
    {
        m_DampingA = dampA;
        m_Mass = mass;


    }
    /// <summary>
    /// Computes the resultant force of the node without taking in count the spring force
    /// </summary>
    /// 
    public void ComputeForces()
    {

        if (!m_Manager.m_AffectedByWind) m_WindForce = Vector3.zero;
        if (m_Manager.m_CanCollide) ComputePenaltyForce();

        m_Force += m_Mass * m_Manager.m_Gravity - m_DampingA * m_Mass * m_Vel + m_WindForce + m_PenaltyForce;

        m_Force *= m_ForceFactor;

        m_PenaltyForce = Vector3.zero;

    }
    private void ComputePenaltyForce()
    {
        float k = 1000f;
        if (m_Manager.m_CollidingObjects.Count == 0) { return; }

        foreach (var obj in m_Manager.m_CollidingObjects)
        {
            
            Collider collider = obj.GetComponent<Collider>();
            if (collider.bounds.Contains(m_Pos))
            {
                Vector3 closest = collider.ClosestPoint(m_Pos);
                Vector3 u = closest - m_Pos;
                float deepness = u.magnitude;
                u.Normalize();

                Vector3 penaltyForce = -k * deepness * u;
                //Debug.Log("Force"+penaltyForce);

                m_PenaltyForce = penaltyForce;
            }


        }

    }
}

/// <summary>
/// Class representing each edge interacting with nodes and aplying compression and traction forces
/// </summary>
public class Spring
{

    public Node m_NodeA, m_NodeB;

    public float m_Length0;
    public float m_Length;
    public float m_Stiffness;
    public float m_Damping;


    public ClothBehaviour m_Manager;

    public Spring(Node nodeA, Node nodeB, ClothBehaviour manager, float stiff, float damping)
    {
        m_NodeA = nodeA;
        m_NodeB = nodeB;
        m_Length0 = (m_NodeA.m_Pos - m_NodeB.m_Pos).magnitude;
        m_Length = m_Length0;
        m_Stiffness = stiff; //boin oi oi oi oing
        m_Damping = damping;
        m_Manager = manager;
    }
    public void UpdateVariables(float stiff)
    {
        m_Stiffness = stiff;
    }

    /// <summary>
    /// Compute de direction of the force
    /// </summary>
    public void ComputeForces()
    {

        Vector3 u = m_NodeA.m_Pos - m_NodeB.m_Pos;
        m_Length = u.magnitude;
        u.Normalize();

        float dampForce = -m_Damping * Vector3.Dot(u, (m_NodeA.m_Vel - m_NodeB.m_Vel));
        float stress = -m_Stiffness * (m_Length - m_Length0) + dampForce;
        Vector3 force = stress * u;
        m_NodeA.m_Force += force;
        m_NodeB.m_Force -= force;


    }
   
}
public class Edge
{
    public int m_A, m_B, m_O;

    public Edge(int a, int b, int o)
    {
        if (a < b)
        {
            m_A = a;
            m_B = b;

        }
        else
        {
            m_B = a;
            m_A = b;
        }
        m_O = o;
    }
}

//public class EdgeComparer : IComparer<Edge>
//{
//    public int Compare(Edge a, Edge b)
//    {
//        if (a.m_A < b.m_A) return -1;
//        else if (b.m_A < a.m_A) return 1;
//        else if (a.m_B < b.m_B) return -1;
//        else if (b.m_B < a.m_B) return 1;
//        else return 0;
//    }
//}

public class EdgeQualityComparer : IEqualityComparer<Edge>
{
    public bool Equals(Edge a, Edge b)
    {
        if (a.m_A == b.m_A && a.m_B == b.m_B || a.m_A == b.m_B && a.m_B == b.m_A) return true;
        else return false;
    }

    public int GetHashCode(Edge e)
    {
        int hcode = e.m_A * e.m_B + e.m_A + e.m_B;
        hcode += (e.m_A % 2 == 0 ? 31 : 0);
        hcode += (e.m_B % 2 == 0 ? 0 : 17);
        return hcode.GetHashCode();
    }
}