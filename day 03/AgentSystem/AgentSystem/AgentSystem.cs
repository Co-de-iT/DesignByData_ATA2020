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
    private void RunScript(bool reset, bool go, List<Point3d> P, List<Vector3d> V, double nR, double coS, double alS, double seS, double seR, ref object Ap, ref object Av, ref object At)
    {
        // <Custom code>

        #region initialization
        // initialization
        if (reset || AgSys == null)
        {
            AgSys = new AgentSystem(P, V);

        }
        #endregion

        // update
        if (go)
        {
            // parameters update
            AgSys.NeighborhoodRadius = nR;
            AgSys.CohesionStrength = coS;
            AgSys.AlignmentStrength = alS;
            AgSys.SeparationStrength = seS;
            AgSys.SeparationRadius = seR;


            // system update
            //AgSys.Update();
            AgSys.UpdateRTree();
            // update solution
            Component.ExpireSolution(true);
        }

        // output
        GH_Point[] ptOut;
        GH_Vector[] velOut;


        AgSys.GetPtsVecs(out ptOut, out velOut);

        Ap = ptOut;
        Av = velOut;
        At = AgSys.GetAgentsTrails();


        // </Custom code>
    }

    // <Custom additional code> 

    //Agent a; // declaration


    public AgentSystem AgSys;

    public class AgentSystem
    {
        // fields
        public List<Agent> Agents;
        public double NeighborhoodRadius;
        public double SeparationRadius;
        public double CohesionStrength;
        public double AlignmentStrength;
        public double SeparationStrength;
        public double MaxSpeed;
        public double BoundingBoxSize;
        public double ContainmentStrength;

        // constructor
        public AgentSystem(List<Point3d> P, List<Vector3d> V)
        {
            Agents = new List<Agent>();

            for (int i = 0; i < P.Count; i++)
            {
                Agent a = new Agent(P[i], V[i]);
                a.Flock = this;

                Agents.Add(a);
            }

            MaxSpeed = 0.3;
            BoundingBoxSize = 30.0;
            ContainmentStrength = 1.0;
        }

        // methods
        public void Update()
        {
            //foreach (Agent a in Agents)
            Parallel.ForEach(Agents, a =>
            {
                ComputeAgentDesiredVelocity(a);
            });

            //foreach (Agent a in Agents)
            Parallel.ForEach(Agents, a =>
            {
                a.UpdateVelocityAndPosition();
            });
        }

        public void UpdateRTree()
        {
            // define and populate the RTree
            RTree rTree = new RTree();

            for (int i = 0; i < Agents.Count; i++)
                rTree.Insert(Agents[i].Position, i);

            // do the actual search

            foreach (Agent a in Agents)
            {
                List<Agent> neighbours = new List<Agent>();

                // here we write the callback function
                EventHandler<RTreeEventArgs> rTreeCallback =
                    (object sender, RTreeEventArgs args) =>
                    {
                        if (Agents[args.Id] != a)
                            neighbours.Add(Agents[args.Id]);
                    };

                // this performs the search
                rTree.Search(new Sphere(a.Position, NeighborhoodRadius), rTreeCallback);

                a.ComputeDesiredVelocity(neighbours);
            }


            //foreach (Agent a in Agents)
            Parallel.ForEach(Agents, a =>
            {
                a.UpdateVelocityAndPosition();
            });
        }

        public List<Agent> FindNeighbours(Agent a)
        {
            List<Agent> neighbours = new List<Agent>();

            foreach (Agent neighbour in Agents)
            {
                if (neighbour != a && neighbour.Position.DistanceTo(a.Position) < NeighborhoodRadius)
                    neighbours.Add(neighbour);
            }

            return neighbours;
        }


        public void ComputeAgentDesiredVelocity(Agent a)
        {
            List<Agent> neighbours = FindNeighbours(a);

            a.ComputeDesiredVelocity(neighbours);
        }

        public List<Polyline> GetAgentsTrails()
        {
            List<Polyline> outTrails = new List<Polyline>();

            foreach (Agent a in Agents)
                outTrails.Add(a.trail);

            return outTrails;
        }

        public void GetPtsVecs(out GH_Point[] pts, out GH_Vector[] vecs)
        {
            pts = new GH_Point[Agents.Count];
            vecs = new GH_Vector[Agents.Count];

            for (int i = 0; i < Agents.Count; i++)
            {
                pts[i] = new GH_Point(Agents[i].Position);
                vecs[i] = new GH_Vector(Agents[i].Velocity);
            }

            // parallel for loop:
            //Parallel.For(0, Agents.Count, i =>
            //{
            //    // parallel loop operations
            //}
            //);
        }
    }

    public class Agent
    {
        //fields
        public Point3d Position;
        public Vector3d Velocity;
        public Vector3d desiredVelocity;
        public AgentSystem Flock;
        public Polyline trail;

        // constructor
        public Agent(Point3d Position, Vector3d Velocity)
        {
            this.Position = Position;
            this.Velocity = Velocity;
            desiredVelocity = this.Velocity;
            //trail = new Polyline(); // empty polyline
            trail = new Polyline { this.Position };

            // array declaration
            //int[] iArray = new int[3];
            //int[] aArray = {0,2,4,6,7};
            // /array declaration

        }
        // methods

        public void ComputeDesiredVelocity(List<Agent> neighbours)
        {
            desiredVelocity = Vector3d.Zero;

            // containment behavior
            Containment();

            // flocking behaviors
            if (neighbours.Count > 0)
            {
                // COHESION BEHAVIOUR

                // find average neighbours position
                Point3d average = new Point3d();

                foreach (Agent neighbour in neighbours)
                    average += neighbour.Position;

                average /= neighbours.Count;

                // go there
                Vector3d cohesion = average - Position;
                desiredVelocity += cohesion * Flock.CohesionStrength;

                // ALIGNMENT BEHAVIOUR
                Vector3d alignment = Vector3d.Zero;

                foreach (Agent neighbour in neighbours)
                    alignment += neighbour.Velocity;

                alignment /= neighbours.Count;

                desiredVelocity += alignment * Flock.AlignmentStrength;

                // SEPARATION BEHAVIOUR
                Vector3d separation = Vector3d.Zero;

                foreach (Agent neighbour in neighbours)
                {
                    double distToNeigh = Position.DistanceTo(neighbour.Position);
                    if (distToNeigh < Flock.SeparationRadius)
                    {
                        Vector3d getAway = Position - neighbour.Position;
                        separation += getAway /= (getAway.Length * distToNeigh);

                    }
                }
                desiredVelocity += separation * Flock.SeparationStrength;

            }

        }

        public void Containment()
        {
            if (Position.X < 0.0)
                desiredVelocity += new Vector3d(-Position.X, 0, 0);
            else if (Position.X > Flock.BoundingBoxSize)
                desiredVelocity += new Vector3d(Flock.BoundingBoxSize - Position.X, 0, 0);

            if (Position.Y < 0.0)
                desiredVelocity += new Vector3d(0, -Position.Y, 0);
            else if (Position.Y > Flock.BoundingBoxSize)
                desiredVelocity += new Vector3d(0, Flock.BoundingBoxSize - Position.Y, 0);

            if (Position.Z < 0.0)
                desiredVelocity += new Vector3d(0, 0, -Position.Z);
            else if (Position.Z > Flock.BoundingBoxSize)
                desiredVelocity += new Vector3d(0, 0, Flock.BoundingBoxSize - Position.Z);

            desiredVelocity *= Flock.ContainmentStrength;
        }

        public void UpdateVelocityAndPosition()
        {
            // steering
            Velocity = 0.97 * Velocity + 0.03 * desiredVelocity;

            // limit velocity
            if (Velocity.Length > Flock.MaxSpeed)
            {
                Velocity.Unitize();
                Velocity *= Flock.MaxSpeed;
            }

            Position += Velocity;
            // adds new position to the trail
            trail.Add(Position);
        }
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
        bool reset = default(bool);
        if (inputs[0] != null)
        {
            reset = (bool)(inputs[0]);
        }

        bool go = default(bool);
        if (inputs[1] != null)
        {
            go = (bool)(inputs[1]);
        }

        List<Point3d> P = null;
        if (inputs[2] != null)
        {
            P = GH_DirtyCaster.CastToList<Point3d>(inputs[2]);
        }
        List<Vector3d> V = null;
        if (inputs[3] != null)
        {
            V = GH_DirtyCaster.CastToList<Vector3d>(inputs[3]);
        }


        //3. Declare output parameters
        object Ap = null;
        object Av = null;


        //4. Invoke RunScript
        RunScript(reset, go, P, V, ref Ap, ref Av);

        try
        {
            //5. Assign output parameters to component...
            if (Ap != null)
            {
                if (GH_Format.TreatAsCollection(Ap))
                {
                    IEnumerable __enum_Ap = (IEnumerable)(Ap);
                    DA.SetDataList(1, __enum_Ap);
                }
                else
                {
                    if (Ap is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(Ap));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, Ap);
                    }
                }
            }
            else
            {
                DA.SetData(1, null);
            }
            if (Av != null)
            {
                if (GH_Format.TreatAsCollection(Av))
                {
                    IEnumerable __enum_Av = (IEnumerable)(Av);
                    DA.SetDataList(2, __enum_Av);
                }
                else
                {
                    if (Av is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(2, (Grasshopper.Kernel.Data.IGH_DataTree)(Av));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(2, Av);
                    }
                }
            }
            else
            {
                DA.SetData(2, null);
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