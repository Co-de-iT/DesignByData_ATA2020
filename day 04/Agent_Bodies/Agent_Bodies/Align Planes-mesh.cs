using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
// <Custom using> 
using System.Threading;
using System.Threading.Tasks;
using System.Drawing;
// </Custom using> 


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance : GH_ScriptInstance
{
    #region Utility functions
    /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
    /// <param name="text">String to print.</param>
    private void Print(string text) { __out.Add(text); }
    /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
    /// <param name="format">String format.</param>
    /// <param name="args">Formatting parameters.</param>
    private void Print(string format, params object[] args) { __out.Add(string.Format(format, args)); }
    /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
    /// <param name="obj">Object instance to parse.</param>
    private void Reflect(object obj) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj)); }
    /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
    /// <param name="obj">Object instance to parse.</param>
    private void Reflect(object obj, string method_name) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj, method_name)); }
    #endregion

    #region Members
    /// <summary>Gets the current Rhino document.</summary>
    private RhinoDoc RhinoDocument;
    /// <summary>Gets the Grasshopper document that owns this script.</summary>
    private GH_Document GrasshopperDocument;
    /// <summary>Gets the Grasshopper script component that owns this script.</summary>
    private IGH_Component Component;
    /// <summary>
    /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
    /// Any subsequent call within the same solution will increment the Iteration count.
    /// </summary>
    private int Iteration;
    #endregion

    /// <summary>
    /// This procedure contains the user code. Input parameters are provided as regular arguments, 
    /// Output parameters as ref arguments. You don't have to assign output parameters, 
    /// they will have a default value.
    /// </summary>
    private void RunScript(bool reset, bool go, List<Point3d> P, List<Vector3d> V, Mesh Ms, Mesh Mv, double pR, double nR, double cS, double aS, double sS, double fS, double meR, double meS, double coS, double mF, ref object Planes, ref object MeshPlanes)
    {
        // <Custom code> 

        // align planes with mesh field - C#
        // code by Alessio Erioli - (c) Co-de-iT 2019 
        //  implementations:
        //  . fixed pts and dirs (for future modularization of the process over larger number of units
        //  . field as static set of pts and vectors (RTree searchable)


        if (P == null || P.Count == 0) return;

        if (reset || aPS == null)
        {
            // passing the essential parameters to the new simulation
            aPS = new AgentPlaneSimulation(P, V);
            MEnvironment = Ms;
            MeshRTree = RTreeFromMesh(Ms);
            TensorField = TensorFieldFromMeshes(Ms, Mv);
            // initializing arrays for export
            gP = new GH_Plane[aPS.agentPlanes.Length];
            gM = new GH_Mesh[aPS.agentPlanes.Length];
        }

        if (go)
        {
            // update parameters
            aPS.PlaneRadius = pR;
            aPS.NeighborhoodRadius = nR;
            aPS.CohesionStrength = cS;
            aPS.AlignmentStrength = aS;
            aPS.SeparationStrength = sS;
            aPS.FieldStrength = fS;
            aPS.MeshSeekRadius = meR;
            aPS.MeshStrength = meS;
            aPS.SeekColorStrength = coS;
            aPS.MaxForce = mF;

            // run simulation
            aPS.Update();

            Component.ExpireSolution(true);
        }

        // extract output geometries and information

        Parallel.For(0, aPS.agentPlanes.Length, i =>
        {
            // extract planes
            gP[i] = new GH_Plane(aPS.agentPlanes[i].PlaneOut());

        });


        Planes = gP;

        // </Custom code> 
    }

    // <Custom additional code> 

    // ............................................................ global variables 
    // declaring global variables for geometry output
    public GH_Plane[] gP;
    public GH_Mesh[] gM;
    public AgentPlaneSimulation aPS;
    public static TensorPoint[] TensorField;
    public static Mesh MEnvironment;
    public static RTree MeshRTree;

    // .................................................................... classes 
    // .............................................................................
    // .............................................................................



    // ...................................................... Simulation ..........
    // ............................................................................
    // ............................................................................

    public class AgentPlaneSimulation
    {
        // ..........................    fields

        //public double cNS;
        //public double cNT;
        public double PlaneRadius;
        public double NeighborhoodRadius;
        public double CohesionStrength;
        public double AlignmentStrength;
        public double SeparationStrength;
        public double FieldStrength;
        public double MaxForce;
        public double MeshSeekRadius;
        public double MeshStrength;
        public double SeekColorStrength;
        public double MaxSpeed;

        public AgentPlane[] agentPlanes;

        // ..........................    constructor

        public AgentPlaneSimulation(List<Point3d> P, List<Vector3d> V, double cNS, double cNT, double pR, double cI, double aI, double sI, double fI, double mF)
        {

            //this.cNS = cNS;
            //this.cNT = cNT;
            this.PlaneRadius = pR;
            this.CohesionStrength = cI;
            this.AlignmentStrength = aI;
            this.SeparationStrength = sI;
            this.FieldStrength = fI;
            this.MaxForce = mF;
            this.MaxSpeed = 0.3;

            // build agent planes array
            agentPlanes = new AgentPlane[P.Count];

            Parallel.For(0, P.Count, i =>
            {
                agentPlanes[i] = new AgentPlane(P[i], V[i], this);
            });

        }

        public AgentPlaneSimulation(List<Point3d> P, List<Vector3d> V)
        {
            // build agent planes array
            agentPlanes = new AgentPlane[P.Count];
            this.MaxSpeed = 0.1;

            Parallel.For(0, P.Count, i =>
            {
                agentPlanes[i] = new AgentPlane(P[i], V[i], this);
            });

        }

        // ..........................    methods

        public void Update()
        {

            // . . . . . . . . . . environmental and stigmergic interactions

            // calculate field influence vector for agents
   
            foreach(AgentPlane ag in agentPlanes)
            {
                // clear neighbour list
                ag.neighbours.Clear();
                ag.neighTens.Clear();

                // calculate field vector from mesh
                // Eventhandler function for RTree search
                EventHandler<RTreeEventArgs> rTreeCallback = (object sender, RTreeEventArgs args) =>
                {
                    ag.neighbours.Add(MEnvironment.Vertices[args.Id]);
                    ag.neighTens.Add(TensorField[args.Id]);
                };

                MeshRTree.Search(new Sphere(MEnvironment.ClosestPoint(ag.O), MeshSeekRadius), rTreeCallback);

                // this resets the acceleration (or desired direction)
                // in case a different update sequence is implemented
                // remember to reset desired before calculating the new iteration
                ag.ResetDesired();
                ag.SeekMeshNeighbours();
                ag.SeekColor();
                ag.ComputeFieldAlign(ag.ComputeFieldVector(),FieldStrength);

            }

            // . . . . . . . . . . peer-to-peer interaction
            FLockRTree();

            // . . . . . . . . . . update position and direction
            UpdateAgentsDirection();

        }


        public void FLockRTree()
        {
            // declare RTree
            RTree rTree = new RTree();

            // populate RTree
            for (int i = 0; i < agentPlanes.Length; i++)
                rTree.Insert(agentPlanes[i].O, i);

            // find neighbours for each agent
            foreach (AgentPlane agent in agentPlanes)
            {
                List<AgentPlane> neighbours = new List<AgentPlane>();

                EventHandler<RTreeEventArgs> rTreeCallback =
                    (object sender, RTreeEventArgs args) =>
                    {
                        if (agentPlanes[args.Id] != agent)
                            neighbours.Add(agentPlanes[args.Id]);
                    };

                rTree.Search(new Sphere(agent.O, NeighborhoodRadius), rTreeCallback);

                // compute desired vector for each agent
                agent.ComputeDesiredNeighbours(neighbours);
            }

        }

        public void UpdateAgentsDirection()
        {
            // here's one of my mods - update positions and velocities in parallel
            Parallel.For(0, agentPlanes.Length, i =>
            {
                // update at max Force
                agentPlanes[i].UpdateDirectionAndPosition();

            });

        }

    }


    // ........................................................... Agent ..........
    // ............................................................................
    // ............................................................................
    /// <summary>
    /// this class contains the Agent Plane definition
    /// </summary>
    public class AgentPlane
    {

        // fields
        public Point3d O;
        public Vector3d X, Y, Z;
        public Vector3d desiredDirection; // desired direction - influenced by environmental field and nearby agents' directions
        public Vector3d desiredPosition; // desired position - influenced by environmental movement constraints (es. surface) and neighbour agents
        public AgentPlaneSimulation agentSim;
        public List<Point3d> neighbours;
        public List<TensorPoint> neighTens;

        // constructor
        public AgentPlane(Point3d O, Vector3d dirX, AgentPlaneSimulation agentSim)
        {
            this.O = O;
            dirX.Unitize();
            this.X = dirX;
            this.agentSim = agentSim;
            neighbours = new List<Point3d>();
            neighTens = new List<TensorPoint>();
        }
        // methods
       
        public void UpdateDirectionAndPosition()
        {
            // update position
            
            Vector3d posUpdate = desiredPosition * agentSim.MaxForce;
            
            if(posUpdate.Length > agentSim.MaxSpeed)
            {
                posUpdate.Unitize();
                posUpdate *= agentSim.MaxSpeed;
            }
            O += posUpdate;
            // update direction
            X = X * (1 - agentSim.MaxForce) + agentSim.MaxForce * desiredDirection;
            X.Unitize();

        }

        public void ResetDesired()
        {
            desiredDirection = Vector3d.Zero;
            desiredPosition = Vector3d.Zero;
        }

        public void ComputeDesiredNeighbours(List<AgentPlane> neighbours)
        {

            // neighbours interaction
            if (neighbours.Count > 0)
            {

                // ............................ alignment behavior

                // align direction with neighbours
                Vector3d align = Vector3d.Zero;

                foreach (AgentPlane neighbour in neighbours)
                    align += neighbour.X;

                align /= neighbours.Count;

                // updates desired direction
                // multiplies alignment vector by alignment intensity factor
                desiredDirection += agentSim.AlignmentStrength * align;

                // ............................ cohesion behavior
                Vector3d cohesion = Vector3d.Zero;
                foreach (AgentPlane neighbour in neighbours)
                    cohesion += (Vector3d)Point3d.Subtract(neighbour.O, O);

                cohesion /= neighbours.Count;

                // updates desired position
                // multiplies cohesion vector by cohesion intensity factor
                desiredPosition += agentSim.CohesionStrength * cohesion;


                // ............................ separation behavior
                Vector3d separation = Vector3d.Zero;
                int sepCount = 0;
                double sepDistSq = agentSim.PlaneRadius * agentSim.PlaneRadius;
                double distSq;
                foreach (AgentPlane neighbour in neighbours)
                {
                    distSq = O.DistanceToSquared(neighbour.O);
                    if (distSq < sepDistSq)
                    {
                        // separation vector is bigger when closer to another agent
                        separation += (Vector3d)Point3d.Subtract(O, neighbour.O) * (sepDistSq - distSq);
                        sepCount++;
                    }
                }

                if (sepCount > 0)
                    separation /= sepCount;

                // updates desired position
                // multiplies separation vector by separation intensity factor
                desiredPosition += agentSim.SeparationStrength * separation;

            }

        }


        public Vector3d ComputeFieldVector()
        {
            Vector3d fieldVector = Vector3d.Zero;
            if (neighTens.Count > 0)
            {
                foreach (TensorPoint nT in neighTens)
                    fieldVector += nT.vector;

                fieldVector /= neighTens.Count;
            }
            else fieldVector = X;

            return fieldVector;
        }
        public void ComputeFieldAlign(Vector3d fieldVector, double fI)
        {
            desiredDirection += fieldVector * fI;
        }

        public void SeekPoint(Point3d target, double intensity)
        {
            Vector3d seek = target - O;
            desiredPosition += seek * intensity;
        }

        //public void SeekMeshClosestPt(Mesh M)
        //{
        //    Point3d mP = M.ClosestPoint(position + velocity * 1.5);
        //    SeekPoint(mP, Agents.MeshStrength);
        //}

        public void SeekMeshNeighbours()
        {
            if (neighbours.Count > 0)
            {

                Point3d average = new Point3d();

                foreach (Point3d n in neighbours)
                    average += n;

                average /= neighbours.Count;

                SeekPoint(average, agentSim.MeshStrength);
            }
        }

        public void SeekColor()
        {

            if (neighTens.Count > 0)
            {
                double bri;
                double MaxBri = -1.0;
                int MaxInd = -1;

                // find point with max intensity
                for (int i = 0; i < neighTens.Count; i++)
                {
                    bri = neighTens[i].scalar; // brightness is 0-1 and RGB 0-255
                    if (bri > MaxBri)
                    {
                        MaxBri = bri;
                        MaxInd = i;
                    }
                }

                    if (MaxBri < 0.2) // move away from extra-dark areas
                {
                    Vector3d seek = (neighTens[MaxInd].position - O) * 0.08 + neighTens[MaxInd].vector * 0.92;
                    desiredPosition += seek * agentSim.SeekColorStrength;
                }

            }
        }

        public Plane PlaneOut()
        {
            Vector3d oV = new Vector3d(O);
            oV.Unitize();

            Vector3d nV = MEnvironment.NormalAt(MEnvironment.ClosestMeshPoint(O, 100));
            //this.Y = Vector3d.CrossProduct(X, oV);
            this.Y = Vector3d.CrossProduct(X, nV);
            Vector3d newX = Vector3d.CrossProduct(nV,this.Y);
            return new Plane(O, newX, Y);
        }

    }

    public class TensorPoint
    {
        public Point3d position;
        public Vector3d vector;
        public double scalar;

        public TensorPoint(Point3d position, Vector3d vector, double scalar)
        {
            this.position = position;
            this.vector = vector;
            this.scalar = scalar;
        }
    }

    public RTree RTreeFromMesh(Mesh M)
    {
        return RTree.CreateFromPointArray(M.Vertices.ToPoint3dArray());
    }

    /// <summary>
    /// This method builds a TensorPoint array from two colored Meshes, one TensorPoint for each Mesh vertex.
    /// </summary>
    /// <remarks>
    /// The Mesh colors are converted into 0 to 1 range from brightness for the scalar field and
    /// to a -1.0 to 1.0 range for Vector X, Y, Z coordinate from 0-255 RGB values respectively.
    /// MScalar and MVector must have the same number of vertices (M.Vertices.Count) and indexed identically
    /// The best strategy is to build MScalar and MVector from the same mesh, just assigning different colors:
    /// . a grayscale pattern for MScalar
    /// . an RBG pattern for MVector
    /// </remarks>
    /// <param name="MScalar">Colored Mesh for scalar field</param>
    /// <param name="MVector">Colored Mesh for Vector field</param>
    /// <returns>An array of TensorPoint, its Length equal to the number of vertices in each Mesh and with corresponding indexes.</returns>
    public TensorPoint[] TensorFieldFromMeshes(Mesh MScalar, Mesh MVector)
    {
        TensorPoint[] MeshField = new TensorPoint[MScalar.Vertices.Count];
        TensorPoint tp;


        for (int i = 0; i < MScalar.Vertices.Count; i++)
        {
            tp = new TensorPoint(MScalar.Vertices[i], VectorFromColor(MVector.VertexColors[i]), MScalar.VertexColors[i].GetBrightness());
            MeshField[i] = tp;
        }

        return MeshField;
    }


    // .................................................................. Utilities 
    // .............................................................................
    // 

    /// <summary>
    /// This function converts a 0-255 RGB color into a -1 to 1 Vector3d
    /// </summary>
    /// <param name="col"></param>
    /// <returns>Vector3d in a -1 to 1 range</returns>
    public Vector3d VectorFromColor(Color col)
    {
        Vector3d vCol = new Vector3d();

        vCol = new Vector3d(
          Map(col.R, 0.0, 255.0, -1.0, 1.0),
          Map(col.G, 0.0, 255.0, -1.0, 1.0),
          Map(col.B, 0.0, 255.0, -1.0, 1.0)
          );
        return vCol;
    }
    public GH_Colour VectorToColor(Vector3d v)
    {
        v.Unitize();
        int red = (int)Math.Floor(Map((float)v.X, -1f, 1f, 0f, 255f));
        int green = (int)Math.Floor(Map((float)v.Y, -1f, 1f, 0f, 255f));
        int blue = (int)Math.Floor(Map((float)v.Z, -1f, 1f, 0f, 255f));
        return new GH_Colour(Color.FromArgb(red, green, blue));
    }

    /// <summary>
    /// This Function maps a value from a source domain to a destination domain
    /// </summary>
    /// <param name="val">the value</param>
    /// <param name="fromMin">low value of source domain</param>
    /// <param name="fromMax">high value of source domain</param>
    /// <param name="toMin">low value of destination domain</param>
    /// <param name="toMax">high value of destination domain</param>
    /// <returns>the remapped value</returns>
    public double Map(double val, double fromMin, double fromMax, double toMin, double toMax)
    {
        return toMin + (val - fromMin) * (toMax - toMin) / (fromMax - fromMin);
    }



    // </Custom additional code> 

    private List<string> __err = new List<string>(); //Do not modify this list directly.
    private List<string> __out = new List<string>(); //Do not modify this list directly.
    private RhinoDoc doc = RhinoDoc.ActiveDoc;       //Legacy field.
    private IGH_ActiveObject owner;                  //Legacy field.
    private int runCount;                            //Legacy field.

    public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA)
    {
        //Prepare for a new run...
        //1. Reset lists
        this.__out.Clear();
        this.__err.Clear();

        this.Component = owner;
        this.Iteration = iteration;
        this.GrasshopperDocument = owner.OnPingDocument();
        this.RhinoDocument = rhinoDocument as Rhino.RhinoDoc;

        this.owner = this.Component;
        this.runCount = this.Iteration;
        this.doc = this.RhinoDocument;

        //2. Assign input parameters
        List<Point3d> P = null;
        if (inputs[0] != null)
        {
            P = GH_DirtyCaster.CastToList<Point3d>(inputs[0]);
        }
        List<Vector3d> V = null;
        if (inputs[1] != null)
        {
            V = GH_DirtyCaster.CastToList<Vector3d>(inputs[1]);
        }
        List<Vector3d> vF = null;
        if (inputs[2] != null)
        {
            vF = GH_DirtyCaster.CastToList<Vector3d>(inputs[2]);
        }
        double pR = default(double);
        if (inputs[3] != null)
        {
            pR = (double)(inputs[3]);
        }

        double cI = default(double);
        if (inputs[4] != null)
        {
            cI = (double)(inputs[4]);
        }

        double aI = default(double);
        if (inputs[5] != null)
        {
            aI = (double)(inputs[5]);
        }

        double sI = default(double);
        if (inputs[6] != null)
        {
            sI = (double)(inputs[6]);
        }

        object fI = default(object);
        if (inputs[7] != null)
        {
            fI = (object)(inputs[7]);
        }

        double mF = default(double);
        if (inputs[8] != null)
        {
            mF = (double)(inputs[8]);
        }



        //3. Declare output parameters
        object Planes = null;


        //4. Invoke RunScript
        RunScript(P, V, vF, pR, cI, aI, sI, fI, mF, ref Planes);

        try
        {
            //5. Assign output parameters to component...
            if (Planes != null)
            {
                if (GH_Format.TreatAsCollection(Planes))
                {
                    IEnumerable __enum_Planes = (IEnumerable)(Planes);
                    DA.SetDataList(1, __enum_Planes);
                }
                else
                {
                    if (Planes is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(Planes));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, Planes);
                    }
                }
            }
            else
            {
                DA.SetData(1, null);
            }

        }
        catch (Exception ex)
        {
            this.__err.Add(string.Format("Script exception: {0}", ex.Message));
        }
        finally
        {
            //Add errors and messages... 
            if (owner.Params.Output.Count > 0)
            {
                if (owner.Params.Output[0] is Grasshopper.Kernel.Parameters.Param_String)
                {
                    List<string> __errors_plus_messages = new List<string>();
                    if (this.__err != null) { __errors_plus_messages.AddRange(this.__err); }
                    if (this.__out != null) { __errors_plus_messages.AddRange(this.__out); }
                    if (__errors_plus_messages.Count > 0)
                        DA.SetDataList(0, __errors_plus_messages);
                }
            }
        }
    }
}