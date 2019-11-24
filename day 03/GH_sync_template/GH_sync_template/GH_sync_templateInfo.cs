using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace GH_sync_template
{
    public class GH_sync_templateInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "GHsynctemplate";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("e4bc1669-d686-4f6f-b623-ddb0fa8f1650");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
