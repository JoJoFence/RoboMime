using System.IO;
using UnityEngine;

namespace AuthenticTeleoperation
{
    internal static class Utilities
    {

        public static void EnsureDirectory( string path )
        {
            try
            {
                if( !string.IsNullOrWhiteSpace( path ) && !Directory.Exists( path ) )
                {
                    Directory.CreateDirectory( path );
                }
            }
            catch
            {
            }
        }


        public static Transform GetDescendantByName( Transform transform, string name )
        {
            Transform childTrans = transform.Find( name );
            if( childTrans != null )
            {
                return childTrans;
            }
            else
            {
                for( int i = 0; i < transform.childCount; ++i )
                {
                    Transform result = GetDescendantByName( transform.GetChild( i ), name );
                    if( result != null )
                    {
                        return result;
                    }
                }
                return null;
            }
        }

    }
}
